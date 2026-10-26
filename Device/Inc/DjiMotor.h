/**
 * 大疆电机设备层（M3508+C620 / M2006+C610）。
 *
 * 这两款电机在总线上是同一套协议：电流模式、收到 0x200（1~4 号）或
 * 0x1FF（5~8 号）里下发、反馈在 0x200+id、帧内布局完全一样。
 * 差别只有几个常数（减速比、命令量程、力矩系数），所以共用一份驱动，
 * 型号差异走 DjiMotor_Model_t 参数表 —— 不为每个型号抄一遍代码。
 *
 * 分层：
 *   Device（本层）  电机语义：反馈解包、工程量换算、命令组帧
 *   BSP/My_CAN      只搬字节，不认识电机
 *   algorithm/PID   只算数学，不认识电机
 *
 * ---------------------------------------------------------------------------
 * 线序（大端！和一般 CAN 报文习惯相反，抄代码时特别注意）
 *
 *   控制帧 0x200 / 0x1FF，8 字节装 4 台，每台 2 字节：
 *     data[0..1] = 1 号电机命令（高字节在前）
 *     data[2..3] = 2 号电机命令
 *     data[4..5] = 3 号电机命令
 *     data[6..7] = 4 号电机命令
 *
 *   反馈帧 0x200+id，7 字节有效：
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

/* 一套协议最多 8 台（1~4 号一组，5~8 号一组） */
#define DJI_MOTOR_MAX   8u

/* 转子编码器分辨率：8192 线/圈（角度反馈 0..8191） */
#define DJI_COUNTS_PER_REV   8192.0f

/* ============================ 型号参数表 ============================
 * 只读，一个型号一行。加新型号就在这里加一条，驱动代码一个字不用改。 */
typedef struct {
    const char *name;

    float   gear_ratio;     /* 减速比。反馈的转速和角度都是【转子侧】，
                               除这个数才是输出轴上的量 */

    int16_t cmd_max;        /* 命令量程。C620 是 ±16384，C610 是 ±10000，
                               填错会让小电调被大电流指令顶爆 */

    float   nm_per_raw;     /* 反馈 raw 电流 → 输出轴 N·m。填 0 表示不换算，
                               此时 torque_nm 恒为 0。这个系数各批次固件不一样，
                               请按手册或实测标定后再填，别照抄。 */
} DjiMotor_Model_t;

extern const DjiMotor_Model_t DJI_MODEL_3508;   /* M3508 + C620 */
extern const DjiMotor_Model_t DJI_MODEL_2006;   /* M2006 + C610 */

/* ============================== 一台电机 ============================== */
typedef struct {
    /* ---- 身份：注册时定死，之后不动 ---- */
    FDCAN_HandleTypeDef    *hfdcan;
    uint8_t                 id;             /* 1..8，反馈 ID = 0x200 + id */
    const DjiMotor_Model_t *model;

    /* ---- 反馈：CAN 中断写，任务只读 ----
     * 中断里只做"存原始值"，不碰浮点、不查表 —— 那些留给任务。 */
    volatile uint16_t raw_angle;            /* 0..8191 */
    volatile int16_t  raw_speed;            /* rpm，转子侧 */
    volatile int16_t  raw_current;
    volatile uint8_t  raw_temp;             /* ℃ */
    volatile uint32_t rx_tick;              /* 最后一帧的时刻 → 掉线判据 */
    volatile uint32_t rx_count;

    /* ---- 工程量：DjiMotor_Update() 里算 ---- */
    int32_t total_counts;                   /* 跨圈累计的转子计数 */
    int32_t prev_raw_angle;
    uint8_t angle_seeded;                   /* 第一帧只记录基准，不产生跳变 */
    float   angle_rad;                      /* 输出轴角度（跨圈连续） */
    float   speed_rad_s;                    /* 输出轴角速度 */
    float   torque_nm;                      /* 输出轴力矩，nm_per_raw 为 0 时恒为 0 */

    /* ---- 命令：任务写，组帧时读 ---- */
    int16_t cmd;                            /* 要下发的原始电流值 */
} DjiMotor_t;

/* ================================ API ================================ */

/* 本机电机实例。挂在哪条总线、哪个 id，在 DjiMotor_Init() 里绑定。
 * 非 static，应用层按需取用，Keil 的 Watch 窗口里也能直接看每台的状态。
 * 这台机构是 3508 还是 6020、挂在 CAN1 还是 CAN2，应用层不用关心 ——
 * 拿到指针就行，换总线只改 DjiMotor_Init() 里那一行。 */
extern DjiMotor_t g_motor_3508_1;      /* 0x201 */
extern DjiMotor_t g_motor_3508_2;      /* 0x202 */
extern DjiMotor_t g_motor_3508_3;      /* 0x203 */
extern DjiMotor_t g_motor_2006_1;      /* 0x204 */
extern DjiMotor_t g_motor_2006_2;      /* 0x205 */

/* 注册失败累计次数（id 越界 / (总线,id) 撞车 / 表满）。
 * 上电后在 Watch 里看一眼：0 = 全部注册成功。非 0 说明有轴永远不会收到
 * 反馈 —— 直接查电调的 id 设置和 DjiMotor_Init() 对不对得上。 */
extern uint32_t g_djimotor_reg_fail;

/* 注册一台电机到内部表。参数不合法（id 越界 / hfdcan 为空 / model 为空）
 * 或该 (总线, id) 已被占用时返回 NULL，并把 g_djimotor_reg_fail 加一。 */
DjiMotor_t *DjiMotor_Register(DjiMotor_t *motor,
                              FDCAN_HandleTypeDef *hfdcan,
                              uint8_t id,
                              const DjiMotor_Model_t *model);

/* 一次把本机所有电机注册好。清单写在 DjiMotor.c 里，改机器人改那里。 */
void DjiMotor_Init(void);

/* 中断上下文：CAN 回调转进来。只存 raw，不做换算。 */
void DjiMotor_OnCanFrame(FDCAN_HandleTypeDef *hfdcan,
                         uint16_t can_id,
                         const uint8_t *data,
                         uint8_t len);

/* 任务上下文：把 raw 换算成 angle_rad / speed_rad_s / torque_nm，
 * 并处理角度回绕。每拍调一次，调完再取角度去做位置环。 */
void DjiMotor_Update(DjiMotor_t *motor);

/* 掉线判定：超过 DJI_MOTOR_TIMEOUT_MS 没收到反馈就算掉线 */
int DjiMotor_IsOnline(const DjiMotor_t *motor);

/* 组帧发送。
 *   group 0 → 0x200，装 id 1~4
 *   group 1 → 0x1FF，装 id 5~8
 * 会顺手把 cmd 按型号量程限幅；掉线的电机按 0 发。
 * 该组一台电机都没注册时返回 HAL_ERROR，不会白发一帧空报文。 */
HAL_StatusTypeDef DjiMotor_SendGroup(FDCAN_HandleTypeDef *hfdcan, uint8_t group);

#endif /* DJIMOTOR_H */
