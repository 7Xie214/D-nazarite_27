/**
 * 电机应用层实现。分层说明见 MotorApp.h 顶部。
 */

#include "MotorApp.h"
#include "PID_Config.h"
#include "fdcan.h"          /* hfdcan1 —— 电机清单里绑总线要用 */

#define TWO_PI      6.28318530718f      /* 输出轴一圈的弧度 */

/* 本机轴表。Device 层不知道它是谁，算法层不知道它连着什么 —— 全在这里对上。 */
static Axis_t s_axis[AXIS_COUNT];

/* 轴 → 控制参数。一根轴一份，因为五台电机装的位置不同、负载不同，
 * 增益和限幅都要各调各的。换机构时这个表和上面的轴表一起改。 */
static const Axis_Cfg_t *const s_axis_cfg[AXIS_COUNT] = {
    &AXIS_CFG_3508_1,
    &AXIS_CFG_3508_2,
    &AXIS_CFG_3508_3,
    &AXIS_CFG_2006_1,
    &AXIS_CFG_2006_2,

};

/* ============================== 初始化 ============================== */

void MotorApp_Init(void)
{
    uint8_t i;

    /* Device 层先把电机注册好（清单和总线绑定在 DjiMotor_Init() 里）。 */
    DjiMotor_Init();

    /* 轴 → 电机。轴 0~2 是 3508，轴 3~4 是 2006；改机构就改这一段。
     * 电机挂在哪条总线上不影响这里 —— 拿到指针就行。
     *
     * 注意这一列绑的是【电机实例】，名字是 g_motor_xxx；
     * 参数是另外一张表 s_axis_cfg，名字是 AXIS_CFG_xxx。
     * 两边都按轴号一一对应，别填串 —— 填串了编不过（类型不同），
     * 但很容易看成"名字长得差不多"而改错地方。 */
    s_axis[0].motor = &g_motor_3508_1;
    s_axis[1].motor = &g_motor_3508_2;
    s_axis[2].motor = &g_motor_3508_3;
    s_axis[3].motor = &g_motor_2006_1;
    s_axis[4].motor = &g_motor_2006_2;

#if (AXIS_COUNT != 5u)
#error "AXIS_COUNT 和下面的轴清单对不上，改机构时两处一起改"
#endif

    for (i = 0u; i < AXIS_COUNT; i++) {
        /* 每根轴套自己那份参数（s_axis_cfg 和 s_axis 一一对应）。
         * 增益是值拷贝进去的，所以各轴的参数虽然来源不同、
         * 状态（积分/微分/上次反馈）本来就各是各的。 */
        PID_Config_Apply(&s_axis[i].cascade, &s_axis[i].traj, s_axis_cfg[i]);
        s_axis[i].state = AXIS_IDLE;
    }
}

Axis_t *MotorApp_GetAxis(uint8_t index)
{
    return (index < AXIS_COUNT) ? &s_axis[index] : NULL;
}

/* ============================== 指令入口 ============================== */

int MotorApp_MoveTurns(Axis_t *axis, float turns)
{
    float from;

    if ((axis == NULL) || (axis->motor == NULL) || (axis->motor->model == NULL)) {
        return MOTORAPP_ERR_PARAM;
    }

    /* 起点必须来自真实反馈。rx_count 为 0 说明 CAN 上一帧都没收到，
     * 这时候 angle_rad 还是 0 —— 拿它当起点，整段行程就全偏了。 */
    if (axis->motor->rx_count == 0u) {
        return MOTORAPP_ERR_NOFB;
    }
    if (DjiMotor_IsOnline(axis->motor) == 0) {
        return MOTORAPP_ERR_OFFLINE;
    }

    /* turns 是【输出轴】圈数。angle_rad 已经是输出轴弧度（DjiMotor_Update
     * 里除过减速比），所以一圈就是 2π —— 不用再乘减速比。 */
    from = axis->motor->angle_rad;

    Traj_Start(&axis->traj, from, from + turns * TWO_PI);
    axis->state = AXIS_MOVING;
    return MOTORAPP_OK;
}

/* ============================== 单轴控制 ============================== */

static void axis_update(Axis_t *ax, float dt)
{
    if (DjiMotor_IsOnline(ax->motor) == 0) {
        /* --- 掉线保护 ---
         * 反馈是冻的，环还在照算：积分会一路堆到饱和，电机回来的那一瞬间
         * 就是一次猛冲。所以停轨迹、复位两个环、命令清零。
         *
         * 复位只做一次（state 判据），每拍都复位会把位置环的保持能力也清掉。 */
        if (ax->state != AXIS_FAULT) {
            Traj_Stop(&ax->traj);
            Cascade_Reset(&ax->cascade, ax->motor->angle_rad, ax->motor->speed_rad_s);
            ax->state = AXIS_FAULT;
        }
        ax->motor->cmd = 0;
        return;
    }

    if (ax->state == AXIS_FAULT) {
        /* --- 回来了 ---
         * 从当前位置重新起步，不去追掉线之前的目标：掉线期间机构可能被
         * 外力挪过，直接追旧目标等于让它自己冲回去，很危险。
         * Traj_Start(from, to) 里 from == to，等于"就停在这儿"。
         *
         * 这里必须再复位一次两个环：掉线（含上电到第一帧之间）那段
         * prev_actual 一直是 0，而真实角度可能已经是几十弧度。不复位的话
         * 恢复后的第一拍微分项是 (0 - 几十)/dt 的天文数字 —— 速度环 Kd
         * 不为 0 时那就是一记实打实的微分冲击。 */
        Traj_Start(&ax->traj, ax->motor->angle_rad, ax->motor->angle_rad);
        Cascade_Reset(&ax->cascade, ax->motor->angle_rad, ax->motor->speed_rad_s);
        ax->state = AXIS_IDLE;
    }

    /* 推进参考轨迹。停着的时候这里无副作用，pos_ref 保持在原地。 */
    Traj_Update(&ax->traj, dt);

    /* 位置环 → 速度环 → 电流命令。
     * pos_ref 是【规划过的】参考位置，不是目标位置 —— 目标已经由规划器
     * 铺成了一条速度受限的曲线，误差才不会被撑大。 */
    ax->motor->cmd = (int16_t)Cascade_Update(&ax->cascade,
                                             ax->motor->angle_rad,
                                             ax->motor->speed_rad_s,
                                             ax->traj.pos_ref,
                                             ax->traj.vel_ref,
                                             dt);

    if ((ax->state == AXIS_MOVING) && (Traj_IsActive(&ax->traj) == 0)) {
        ax->state = AXIS_IDLE;      /* 到位 */
    }
}

/* ============================== 组帧发送 ============================== */

/* 按总线去重后，每条总线各发自己的 0x200 / 0x1FF。
 * 全在同一路 CAN 上时，这段等价于发两帧（其中一组往往是空的，
 * DjiMotor_SendGroup 空组会返回 HAL_ERROR，不会白发）。 */
static void send_all_buses(void)
{
    FDCAN_HandleTypeDef *bus[AXIS_COUNT];
    uint8_t n = 0u;
    uint8_t i, j;

    for (i = 0u; i < AXIS_COUNT; i++) {
        FDCAN_HandleTypeDef *h = s_axis[i].motor->hfdcan;
        uint8_t seen = 0u;

        for (j = 0u; j < n; j++) {
            if (bus[j] == h) {
                seen = 1u;
                break;
            }
        }
        if (seen == 0u) {
            bus[n] = h;
            n++;
        }
    }

    for (i = 0u; i < n; i++) {
        (void)DjiMotor_SendGroup(bus[i], 0u);   /* 0x200，id 1~4 */
        (void)DjiMotor_SendGroup(bus[i], 1u);   /* 0x1FF，id 5~8 */
    }
}

/* ============================== 主循环 ============================== */

void MotorApp_Update(float dt)
{
    uint8_t i;

    /* 1) 先把所有轴的反馈换算完。
     *    必须排在控制之前，而且要全部做完再开始控 —— 这样每根轴用的都是
     *    同一拍的反馈，轴和轴之间不会差半拍。 */
    for (i = 0u; i < AXIS_COUNT; i++) {
        DjiMotor_Update(s_axis[i].motor);
    }

    /* 2) 逐轴算控制量 */
    for (i = 0u; i < AXIS_COUNT; i++) {
        axis_update(&s_axis[i], dt);
    }

    /* 3) 统一发出。命令必须全部算完再发 —— 0x200 一帧装 4 台电机，
     *    边算边发会拆成两帧，中间的半拍状态没有意义。 */
    send_all_buses();
    
}
