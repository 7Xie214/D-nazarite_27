/**
 * 大疆电机设备层（M3508+C620 / M2006+C610 / GM6020 电压控制）。
 *
 * 3508/2006：电流命令，控制帧 0x200 / 0x1FF，反馈 0x200+id。
 * GM6020：电压命令，控制帧 0x1FF / 0x2FF，反馈 0x204+id。
 * 型号参数保存各自的 ID 范围、反馈基址和控制帧，共用反馈解析与组帧代码。
 *
 * 分层：
 *   Device（本层）  电机语义：反馈解包、工程量换算、命令组帧
 *   BSP/My_CAN      只搬字节，不认识电机
 *   algorithm/PID   只算数学，不认识电机
 *
 * ---------------------------------------------------------------------------
 * 线序（高字节在前）
 *
 *   每个控制帧 8 字节、4 个槽，每台 2 字节。
 *   各型号 id 1~4 用第一组，id 5~8 用第二组（GM6020 最大 id 为 7）。
 *
 *   反馈帧前 7 字节有效：
 *     data[0..1] = 机械角度   0..8191，转子转一圈
 *     data[2..3] = 转速       rpm（也是转子侧，不是输出轴）
 *     data[4..5] = 实际电流   原始值
 *     data[6]    = 温度       ℃
 *
 * ---------------------------------------------------------------------------
 * 【发送是按组、不是按台】0x200 一帧塞 4 台，所以接口是
 * DjiMotor_SendGroup()，不是 Motor_Send(&m1)。一台一帧在物理上就发不出去。
 *
 * 【角度必须累计】raw_angle 是 0..8191 循环的，直接当绝对值用，转过一圈
 * 就跳回 0。DjiMotor_Update() 里用差分累计算成 total_counts，跨圈连续，
 * 位置环要的就是这个。
 *
 * 【掉线不会报错】CAN 电机丢了不会有任何异常，raw_* 就静静停在最后一个值。
 * 拿冻结的反馈继续跑位置环，积分会一路堆到饱和，电机回来那一瞬间猛冲。
 * 用 DjiMotor_IsOnline() 判，超时就把该轴的命令清零 + 复位对应 PID。
 * DjiMotor_SendGroup() 内部已经替你把这台按 0 发，但复位 PID 是应用层的事。
 */

#ifndef DJIMOTOR_H
#define DJIMOTOR_H

#include "main.h"

/* 所有 CAN 总线合计的注册容量，不是电调 ID 上限。 */
#define DJI_MOTOR_MAX   13u

/* 转子编码器分辨率：8192 线/圈（角度反馈 0..8191） */
#define DJI_COUNTS_PER_REV   8192.0f

/* ============================ 型号参数表 ============================
 * 只读。以下三款共用帧内布局，ID 和工程量差异放在参数表中。 */
typedef struct {
    const char *name;

    float   gear_ratio;     /* 减速比。反馈的转速和角度都是【转子侧】，
                               除这个数才是输出轴上的量 */

    int16_t cmd_max;        /* C620 ±16384，C610 ±10000，GM6020 电压 ±30000 */

    float   nm_per_raw;     /* 反馈 raw 电流 → 输出轴 N·m。填 0 表示不换算，
                               此时 torque_nm 恒为 0。这个系数各批次固件不一样，
                               请按手册或实测标定后再填，别照抄。 */

    uint16_t feedback_base; /* 反馈 ID = feedback_base + 电调 id */
    uint16_t command_id[2]; /* id 1~4、id 5~8 的控制帧 ID */
    uint8_t  max_id;        /* 3508/2006: 8；GM6020: 7，与注册容量无关 */
} DjiMotor_Model_t;

extern const DjiMotor_Model_t DJI_MODEL_3508;   /* M3508 + C620 */
extern const DjiMotor_Model_t DJI_MODEL_2006;   /* M2006 + C610 */
extern const DjiMotor_Model_t DJI_MODEL_6020;   /* GM6020，电压控制 ±30000 */

/* ============================== 一台电机 ============================== */
typedef struct {
    /* ---- 身份：注册时定死，之后不动 ---- */
    FDCAN_HandleTypeDef    *hfdcan;
    uint8_t                 id;             /* 电调硬件 ID，范围由型号决定 */
    const DjiMotor_Model_t *model;

    /* ---- 反馈：CAN 中断写，任务只读 ----
     * 中断里只做"存原始值"，不碰浮点、不查表 —— 那些留给任务。 */
    volatile uint16_t raw_angle;            /* 0..8191 */
    volatile int16_t  raw_speed;            /* rpm，转子侧 */
    volatile int16_t  raw_current;
    volatile uint8_t  raw_temp;             /* ℃ */
    volatile uint32_t rx_tick;              /* 最后一帧 HAL_GetTick()，单位 ms */
    volatile uint32_t rx_count;

    /* ---- 工程量：DjiMotor_Update() 里算 ---- */
    int32_t total_counts;                   /* 跨圈累计的转子计数 */
    int32_t prev_raw_angle;
    uint8_t angle_seeded;                   /* 第一帧只记录基准，不产生跳变 */
    float   angle_rad;                      /* 输出轴角度（跨圈连续） */
    float   speed_rad_s;                    /* 输出轴角速度 */
    float   torque_nm;                      /* 输出轴力矩，nm_per_raw 为 0 时恒为 0 */

    /* ---- 命令：任务写，组帧时读 ---- */
    int16_t cmd;                            /* 原始命令：3508/2006 电流，6020 电压 */
} DjiMotor_t;

/* ================================ API ================================ */

/* 本机电机实例。挂在哪条总线、哪个 id，在 DjiMotor_Init() 里绑定。
 * 非 static，应用层按需取用，Keil 的 Watch 窗口里也能直接看每台的状态。
 * 这台机构是 3508 还是 6020、挂在 CAN1 还是 CAN2，应用层不用关心 ——
 * 拿到指针就行，换总线只改 DjiMotor_Init() 里那一行。 */
/* 名字末尾的数字是实例编号，实际硬件 ID 以注册参数为准。 */
extern DjiMotor_t g_motor_3508_1;      /* 上层：CAN3 ID5 */
extern DjiMotor_t g_motor_3508_2;      /* 上层：CAN3 ID2 */
extern DjiMotor_t g_motor_3508_3;      /* 上层：CAN3 ID1 */
extern DjiMotor_t g_motor_3508_4;      /* 左前行走：CAN1 ID1 */
extern DjiMotor_t g_motor_3508_5;      /* 右前行走：CAN2 ID4 */
extern DjiMotor_t g_motor_3508_6;      /* 左后行走：CAN1 ID2 */
extern DjiMotor_t g_motor_3508_7;      /* 右后行走：CAN2 ID3 */
extern DjiMotor_t g_motor_2006_1;      /* 上层：CAN3 ID4 */
extern DjiMotor_t g_motor_2006_2;      /* 上层：CAN3 ID3 */
extern DjiMotor_t g_motor_6020_1;      /* 左前转向：CAN1 ID1 */
extern DjiMotor_t g_motor_6020_2;      /* 左后转向：CAN1 ID2 */
extern DjiMotor_t g_motor_6020_3;      /* 右后转向：CAN2 ID3 */
extern DjiMotor_t g_motor_6020_4;      /* 右前转向：CAN2 ID4 */

/* 注册失败累计次数（id 越界 / 反馈 ID 冲突 / 实例重复 / 表满）。
 * 上电后在 Watch 里看一眼：0 = 全部注册成功。非 0 说明有轴永远不会收到
 * 反馈 —— 直接查电调的 id 设置和 DjiMotor_Init() 对不对得上。 */
extern uint32_t g_djimotor_reg_fail;

/* 启动时注册一次。参数不合法、实例重复、表满，或同总线反馈 ID 冲突时
 * 返回 NULL，并把 g_djimotor_reg_fail 加一。
 * 同路混用型号时按实际反馈 ID 判断冲突，例如 3508 id5 与 6020 id1 冲突。 */
DjiMotor_t *DjiMotor_Register(DjiMotor_t *motor,
                              FDCAN_HandleTypeDef *hfdcan,
                              uint8_t id,
                              const DjiMotor_Model_t *model);

/* main 在外设初始化后、CAN/RTOS 启动前调用一次，注册本机全部电机。
 * 清单写在 DjiMotor.c 里；应用层初始化不得重复调用。 */
void DjiMotor_Init(void);

/* 中断上下文：CAN 回调转进来。只存 raw，不做换算。 */
void DjiMotor_OnCanFrame(FDCAN_HandleTypeDef *hfdcan,
                         uint16_t can_id,
                         const uint8_t *data,
                         uint8_t len);

/* 任务上下文：把 raw 换算成 angle_rad / speed_rad_s / torque_nm，
 * 并处理角度回绕。每拍调一次，调完再取角度去做位置环。 */
void DjiMotor_Update(DjiMotor_t *motor);

/* 掉线判定：用 HAL 毫秒时钟，超过 50 ms 没收到反馈就算掉线。 */
int DjiMotor_IsOnline(const DjiMotor_t *motor);

/* 组帧发送。
 *   group 0 → 0x200，装 3508/2006 id 1~4
 *   group 1 → 0x1FF，装 3508/2006 id 5~8，以及 GM6020 id 1~4
 *   group 2 → 0x2FF，装 GM6020 id 5~7
 * 按实际 CAN 帧合并，不会将两种型号分别发帧而相互覆盖槽位。
 * 会顺手把 cmd 按型号量程限幅；掉线的电机按 0 发。
 * 该组一台电机都没注册时返回 HAL_ERROR，不会白发一帧空报文。 */
HAL_StatusTypeDef DjiMotor_SendGroup(FDCAN_HandleTypeDef *hfdcan, uint8_t group);

#endif /* DJIMOTOR_H */
