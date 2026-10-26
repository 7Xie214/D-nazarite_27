/**
 * 大疆电机驱动实现。
 *
 * 分四层：
 *   ① 注册表      按 (总线, id) 找电机
 *   ② OnCanFrame  中断里：存 raw + 记时间戳
 *   ③ Update      任务里：角度累计 + 工程量换算
 *   ④ SendGroup   任务里：打包成 0x200 / 0x1FF 发出去
 *
 * ① 不知道协议，② 不做数学，③ 不发报文，④ 不碰状态。各管一段。
 */

#include "DjiMotor.h"
#include "My_CAN.h"
#include "fdcan.h"          /* hfdcan1 */

#include "FreeRTOS.h"
#include "cmsis_os2.h"      /* osKernelGetTickCount */

/* ========================= 型号参数表 =========================
 * 加新型号在这里加一条，别的地方不用动。
 *
 * ⚠️ nm_per_raw 我填的是 0（= 不换算，torque_nm 恒为 0）。
 *    这个系数跟 C620/C610 的固件版本走，各批次不一样，我给不出可靠值 ——
 *    拿着手册的力矩常数，或者实测标定之后再填。填了这里力矩就能用了。 */

const DjiMotor_Model_t DJI_MODEL_3508 = {
    .name       = "M3508",
    .gear_ratio = 3591.0f / 187.0f,     /* ≈ 19.203 */
    .cmd_max    = 16384,                /* C620 的量程 */
    .nm_per_raw = 0.0f,                 /* TODO: 标定后填 */
};

const DjiMotor_Model_t DJI_MODEL_2006 = {
    .name       = "M2006",
    .gear_ratio = 36.0f,
    .cmd_max    = 10000,                /* C610 的量程，比 C620 小 */
    .nm_per_raw = 0.0f,                 /* TODO: 标定后填 */
};

/* ========================= 掉线判据 ========================= */

#define DJI_MOTOR_TIMEOUT_MS   50u
#define DJI_TIMEOUT_TICKS      ((uint32_t)DJI_MOTOR_TIMEOUT_MS * (uint32_t)configTICK_RATE_HZ / 1000u)

/* ========================= ① 注册表 =========================
 * 定长数组，不是链表 —— 查找有上界，缓存友好。
 * 按 (总线, id) 匹配：同一路 CAN 上 id 唯一，但不同总线可以有同 id 的电机，
 * 所以总线也要一起比。8 项线性扫描，在中断里也就是几十纳秒。 */

static DjiMotor_t *s_table[DJI_MOTOR_MAX];

/* 注册失败次数。上电后在 Keil 的 Watch 里看一眼就行：
 *   0   = 五台全部注册成功
 *   非 0 = 有电机没注册上（id 越界 / (总线,id) 撞车 / 表满），
 *          那台轴会永远收不到反馈 —— 现象是 MoveTurns 一直返回
 *          MOTORAPP_ERR_NOFB，但 CAN 上看着一切正常，很难查。
 * 最常见的起因：两台电调的 id 被设成了同一个（硬件那边的事），
 * 或者固件这边 DjiMotor_Init 里的号和 main.c 的滤波器对不上。 */
uint32_t g_djimotor_reg_fail;

static DjiMotor_t *dji_find(FDCAN_HandleTypeDef *hfdcan, uint8_t id)
{
    uint8_t i;

    for (i = 0u; i < DJI_MOTOR_MAX; i++) {
        DjiMotor_t *m = s_table[i];
        if ((m != NULL) && (m->hfdcan == hfdcan) && (m->id == id)) {
            return m;
        }
    }
    return NULL;
}

DjiMotor_t *DjiMotor_Register(DjiMotor_t *motor,
                              FDCAN_HandleTypeDef *hfdcan,
                              uint8_t id,
                              const DjiMotor_Model_t *model)
{
    uint8_t i;

    if ((motor == NULL) || (hfdcan == NULL) || (model == NULL)) {
        g_djimotor_reg_fail++;
        return NULL;
    }
    if ((id < 1u) || (id > DJI_MOTOR_MAX)) {
        g_djimotor_reg_fail++;
        return NULL;
    }
    if (dji_find(hfdcan, id) != NULL) {
        g_djimotor_reg_fail++;
        return NULL;                    /* (总线, id) 撞车 */
    }

    for (i = 0u; i < DJI_MOTOR_MAX; i++) {
        if (s_table[i] == NULL) {
            s_table[i] = motor;

            motor->hfdcan          = hfdcan;
            motor->id              = id;
            motor->model           = model;
            motor->raw_angle       = 0u;
            motor->raw_speed       = 0;
            motor->raw_current     = 0;
            motor->raw_temp        = 0u;
            motor->rx_tick         = 0u;
            motor->rx_count        = 0u;
            motor->total_counts    = 0;
            motor->prev_raw_angle  = 0;
            motor->angle_seeded    = 0u;
            motor->angle_rad       = 0.0f;
            motor->speed_rad_s     = 0.0f;
            motor->torque_nm       = 0.0f;
            motor->cmd             = 0;

            return motor;
        }
    }
    g_djimotor_reg_fail++;
    return NULL;                        /* 表满 */
}

/* ========================= ② 中断：存 raw =========================
 * 只做四件事：拷 4 个原始值、记时间戳、计数。
 * 浮点换算、角度回绕全留给任务 —— 中断里那些都是白烧时间。
 *
 * 注意是【大端】：高字节在前。DJI 这套帧和一般 CAN 报文的习惯相反，
 * 抄别的库时最容易在这里翻车。 */

void DjiMotor_OnCanFrame(FDCAN_HandleTypeDef *hfdcan,
                         uint16_t can_id,
                         const uint8_t *data,
                         uint8_t len)
{
    DjiMotor_t *m;
    uint8_t id;

    if ((data == NULL) || (len < 7u)) {
        return;                         /* 反馈帧有效长度是 7 */
    }
    if ((can_id < 0x201u) || (can_id > 0x208u)) {
        return;                         /* 不是电机反馈 */
    }

    id = (uint8_t)(can_id - 0x200u);
    m  = dji_find(hfdcan, id);
    if (m == NULL) {
        return;                         /* 这台没注册，静默丢 */
    }

    m->raw_angle   = (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    m->raw_speed   = (int16_t)(((int16_t)data[2] << 8) | (int16_t)data[3]);
    m->raw_current = (int16_t)(((int16_t)data[4] << 8) | (int16_t)data[5]);
    m->raw_temp    = data[6];

    m->rx_tick  = osKernelGetTickCount();
    m->rx_count++;
}

/* ========================= ③ 任务：换算 ========================= */

int DjiMotor_IsOnline(const DjiMotor_t *motor)
{
    if ((motor == NULL) || (motor->rx_count == 0u)) {
        return 0;                       /* 一帧都没收到过 */
    }
    return ((osKernelGetTickCount() - motor->rx_tick) <= DJI_TIMEOUT_TICKS) ? 1 : 0;
}

void DjiMotor_Update(DjiMotor_t *motor)
{
    int32_t raw, delta;
    float   rotor_rev;

    /* model 为空 = 没注册过。全局那些电机结构体上电都是 0，
     * DjiMotor_Init() 之前就调本函数会踩到空指针，挡一下。 */
    if ((motor == NULL) || (motor->model == NULL)) {
        return;
    }

    raw = (int32_t)motor->raw_angle;

    /* --- 角度回绕 ---
     * raw 在 0..8191 之间循环，转过一圈就跳回 0。直接当绝对值用会看到
     * 一次 8191 的跳变，差值法 + 半圈判决把它掰回连续量。
     *
     * 第一帧只记基准：否则 prev 是 0，而实际可能在 8000，白白产生一次假跳变。 */
    if (motor->angle_seeded == 0u) {
        motor->prev_raw_angle = raw;
        motor->angle_seeded   = 1u;
    }

    delta = raw - motor->prev_raw_angle;
    if (delta >  (int32_t)(DJI_COUNTS_PER_REV / 2.0f)) {
        delta -= (int32_t)DJI_COUNTS_PER_REV;      /* 正向跨零 */
    } else if (delta < -(int32_t)(DJI_COUNTS_PER_REV / 2.0f)) {
        delta += (int32_t)DJI_COUNTS_PER_REV;      /* 反向跨零 */
    }
    motor->prev_raw_angle = raw;
    motor->total_counts  += delta;

    /* --- 转成输出轴工程量 ---
     * 编码器和转速反馈都在【转子侧】，要除以减速比才是输出轴上的量。 */
    rotor_rev        = (float)motor->total_counts / DJI_COUNTS_PER_REV;
    motor->angle_rad = rotor_rev / motor->model->gear_ratio * 6.28318530718f;

    motor->speed_rad_s = ((float)motor->raw_speed / motor->model->gear_ratio)
                         * 6.28318530718f / 60.0f;

    /* nm_per_raw 填 0 时这里恒为 0 —— 表示"没标定，别信这个数" */
    motor->torque_nm = (float)motor->raw_current * motor->model->nm_per_raw;
}

/* ========================= ④ 任务：组帧发送 ========================= */

HAL_StatusTypeDef DjiMotor_SendGroup(FDCAN_HandleTypeDef *hfdcan, uint8_t group)
{
    uint8_t  data[8];
    uint8_t  base, i;
    uint16_t can_id;
    int      any = 0;

    if (hfdcan == NULL) {
        return HAL_ERROR;
    }
    if (group == 0u) {
        can_id = 0x200u;
        base   = 1u;                    /* 装 id 1~4 */
    } else if (group == 1u) {
        can_id = 0x1FFu;
        base   = 5u;                    /* 装 id 5~8 */
    } else {
        return HAL_ERROR;
    }

    for (i = 0u; i < 8u; i++) {
        data[i] = 0u;                   /* 空槽位补 0，电机收到 0 = 不出力 */
    }

    for (i = 0u; i < DJI_MOTOR_MAX; i++) {
        DjiMotor_t *m = s_table[i];
        int32_t     v;
        uint8_t     slot;

        if ((m == NULL) || (m->hfdcan != hfdcan)) {
            continue;
        }
        if ((m->id < base) || (m->id >= (uint8_t)(base + 4u))) {
            continue;
        }

        /* 掉线：按 0 发。电机侧可能还在按上一帧的指令转，
         * 但至少我们不再往上加力 —— 拿冻结的反馈闭环比停手危险得多。 */
        v = DjiMotor_IsOnline(m) ? (int32_t)m->cmd : 0;

        if (v >  (int32_t)m->model->cmd_max) {
            v =  (int32_t)m->model->cmd_max;
        }
        if (v < -(int32_t)m->model->cmd_max) {
            v = -(int32_t)m->model->cmd_max;
        }

        slot = (uint8_t)(m->id - base);         /* 0..3 → 帧内第几个 */
        data[slot * 2u]      = (uint8_t)((v >> 8) & 0xFF);      /* 高字节在前 */
        data[slot * 2u + 1u] = (uint8_t)(v & 0xFF);

        any = 1;
    }

    if (any == 0) {
        return HAL_ERROR;               /* 这一组一台都没注册，别发空帧 */
    }

    return BSP_CAN_Send(hfdcan, can_id, data, 8u);
}

/* ========================= 本机电机清单 =========================
 * 现在的配置：CAN1 上 3 台 M3508（id 1~3）+ 2 台 M2006（id 4~5）。
 *
 * id 是【电调自己的设置】（C620/C610 在上位机软件里配），固件这边的号
 * 只是跟着填，不是我们定的。两边必须一致：不一致时反馈会解到别人身上
 * （两台对调），或者那台一点反馈都收不到（被 main.c 的滤波器挡掉）。
 *
 * 改 id 只动两处：这里的 DjiMotor_Register 参数，和 main.c 的 can1_filter。
 * 驱动、轴表、算法都不用动 —— Register/SendGroup 都是把 id 当参数收的。
 *
 * 非 static：Keil 的 Watch 窗口里能直接看每台的状态，调车时方便。 */

DjiMotor_t g_motor_3508_1;      /* 0x201 */
DjiMotor_t g_motor_3508_2;      /* 0x202 */
DjiMotor_t g_motor_3508_3;      /* 0x203 */
DjiMotor_t g_motor_2006_1;      /* 0x204 */
DjiMotor_t g_motor_2006_2;      /* 0x205 */

void DjiMotor_Init(void)
{
    (void)DjiMotor_Register(&g_motor_3508_1, &hfdcan1, 1u, &DJI_MODEL_3508);
    (void)DjiMotor_Register(&g_motor_3508_2, &hfdcan1, 2u, &DJI_MODEL_3508);
    (void)DjiMotor_Register(&g_motor_3508_3, &hfdcan1, 3u, &DJI_MODEL_3508);
    (void)DjiMotor_Register(&g_motor_2006_1, &hfdcan1, 4u, &DJI_MODEL_2006);
    (void)DjiMotor_Register(&g_motor_2006_2, &hfdcan1, 5u, &DJI_MODEL_2006);
}

/* ========================= 接管 CAN 接收 =========================
 * BSP 里 BSP_CAN_OnReceive 是 __weak 空实现，这里覆盖它。
 * 以后加了非电机的 CAN 设备（姿态传感器、上位机指令之类），
 * 在这个函数里按 id 分发出去就行 —— 电机这条路走 DjiMotor_OnCanFrame。 */

void BSP_CAN_OnReceive(FDCAN_HandleTypeDef *hfdcan,
                       uint16_t can_id,
                       const uint8_t *data,
                       uint8_t len)
{
    DjiMotor_OnCanFrame(hfdcan, can_id, data, len);
}
