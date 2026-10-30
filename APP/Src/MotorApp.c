/**
 * 电机应用层实现。分层说明见 MotorApp.h 顶部。
 */

#include "MotorApp.h"
#include "PID_Config.h"
#include "fdcan.h"          /* hfdcan1 —— 电机清单里绑总线要用 */
#include <math.h>
#include <string.h>

#define TWO_PI      6.28318530718f      /* 输出轴一圈的弧度 */

/* 本机轴表。Device 层不知道它是谁，算法层不知道它连着什么 —— 全在这里对上。 */
static Axis_t s_axis[AXIS_COUNT];
static Traj_t s_sync_traj;
static float s_sync_from[3];
static float s_sync_delta[3];
static uint8_t s_sync_running;
static Traj_t s_pair_traj;
static MotorApp_2006PairStatus_t s_pair_status;
static float s_pair_from[2], s_pair_speed[2];
static uint32_t s_pair_timing_last_tick;
/* 统一正方向 = ID3负转、ID4正转；数组始终按 ID3、ID4。 */
static const uint8_t s_pair_axis[2] = {4u, 3u};
static const float s_pair_direction[2] = {-1.0f, 1.0f};
static float pair_limit(float value, float limit);

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

    s_sync_running = 0u;
    Traj_Init(&s_sync_traj, 0.0f, 0.0f);
    memset(&s_pair_status, 0, sizeof(s_pair_status));
    Traj_Init(&s_pair_traj, MOTORPAIR_CFG_2006.traj_v_max, MOTORPAIR_CFG_2006.traj_a_max);

    /* main 已在启动 CAN 前调用 DjiMotor_Init；这里只初始化上层控制器。 */

    /* 机构轴 → 电机。轴 0~2 是 3508，轴 3~4 是 2006；改机构就改这一段。
     * 上层 5 台均在 CAN3，具体硬件 ID 在 DjiMotor_Init 中配置。
     * 底盘使用另外 8 个实例，不接入这里的爬楼轨迹控制。
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
        s_axis[i].settled_s = 0.0f;
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

    if ((s_sync_running != 0u) &&
        ((axis == &s_axis[0]) || (axis == &s_axis[1]) || (axis == &s_axis[2]))) {
        return MOTORAPP_ERR_BUSY;
    }
    if ((s_pair_status.state != MOTORAPP_PAIR_IDLE) &&
        ((axis == &s_axis[3]) || (axis == &s_axis[4]))) {
        return MOTORAPP_ERR_BUSY;
    }

    /* 外部给转子圈数；角度反馈已经是输出轴弧度，只在这里除一次减速比。 */
    from = axis->motor->angle_rad;

    Traj_Start(&axis->traj, from, from + turns * TWO_PI / axis->motor->model->gear_ratio);
    axis->settled_s = 0.0f;
    axis->state = AXIS_MOVING;
    return MOTORAPP_OK;
}

int MotorApp_Move3508TurnsSync(const float turns[3], float rate, float accel)
{
    uint8_t i;
    uint8_t has_travel = 0u;

    if ((turns == NULL) || !(rate > 0.0f) || !(accel > 0.0f)) {
        return MOTORAPP_ERR_PARAM;
    }
    if (s_sync_running != 0u) { return MOTORAPP_ERR_BUSY; }
    /* 检查完三台再修改任何轨迹，避免一台先跑、另一台尚未收到反馈。 */
    for (i = 0u; i < 3u; i++) {
        if ((s_axis[i].motor == NULL) || (s_axis[i].motor->model == NULL)) { return MOTORAPP_ERR_PARAM; }
        if (s_axis[i].motor->rx_count == 0u) { return MOTORAPP_ERR_NOFB; }
        if (DjiMotor_IsOnline(s_axis[i].motor) == 0) { return MOTORAPP_ERR_OFFLINE; }
        if (s_axis[i].state == AXIS_MOVING) { return MOTORAPP_ERR_BUSY; }
    }
    for (i = 0u; i < 3u; i++) {
        s_sync_from[i] = s_axis[i].motor->angle_rad;
        s_sync_delta[i] = turns[i] * TWO_PI / s_axis[i].motor->model->gear_ratio;
        if (turns[i] != 0.0f) { has_travel = 1u; }
        Traj_Start(&s_axis[i].traj, s_sync_from[i], s_sync_from[i] + s_sync_delta[i]);
        s_axis[i].settled_s = 0.0f;
        s_axis[i].state = AXIS_MOVING;
    }
    Traj_Init(&s_sync_traj, rate, accel);
    Traj_Start(&s_sync_traj, 0.0f, has_travel != 0u ? 1.0f : 0.0f);
    s_sync_running = 1u;
    return MOTORAPP_OK;
}

const MotorApp_2006PairStatus_t *MotorApp_Get2006PairStatus(void)
{
    return &s_pair_status;
}

int MotorApp_Move2006Pair(float turns)
{
    uint8_t i;
    if (!isfinite(turns)) { return MOTORAPP_ERR_PARAM; }
    if (s_pair_status.state == MOTORAPP_PAIR_FAULT) { return MOTORAPP_ERR_FAULT; }
    if (s_pair_status.state == MOTORAPP_PAIR_MOVING) { return MOTORAPP_ERR_BUSY; }
    /* 先检查两台，检查失败不能让其中一台先起步。 */
    for (i = 0u; i < 2u; i++) {
        Axis_t *a = &s_axis[s_pair_axis[i]];
        if (a->motor->rx_count == 0u) { return MOTORAPP_ERR_NOFB; }
        if (DjiMotor_IsOnline(a->motor) == 0) { return MOTORAPP_ERR_OFFLINE; }
        if (a->state == AXIS_MOVING) { return MOTORAPP_ERR_BUSY; }
    }
    memset(&s_pair_status, 0, sizeof(s_pair_status));
    s_pair_status.state = MOTORAPP_PAIR_MOVING;
    s_pair_status.time_scale = 1.0f;
    s_pair_status.timing.start_ms = HAL_GetTick();
    s_pair_timing_last_tick = s_pair_status.timing.start_ms;
    s_pair_status.timing.phase = 1u;
    s_pair_status.timing.status = 1u;
    s_pair_status.timing.planner_ms = UINT32_MAX;
    s_pair_status.timing.reach_75_ms = UINT32_MAX;
    for (i = 0u; i < 2u; i++) {
        s_pair_status.timing.first_motion_ms[i] = UINT32_MAX;
        s_pair_status.timing.first_in_tol_ms[i] = UINT32_MAX;
    }
    Traj_Init(&s_pair_traj, MOTORPAIR_CFG_2006.traj_v_max, MOTORPAIR_CFG_2006.traj_a_max);
    Traj_Start(&s_pair_traj, 0.0f, turns * TWO_PI / s_axis[3].motor->model->gear_ratio);
    for (i = 0u; i < 2u; i++) {
        Axis_t *a = &s_axis[s_pair_axis[i]];
        s_pair_from[i] = a->motor->angle_rad;
        s_pair_speed[i] = pair_limit(a->motor->speed_rad_s, a->cascade.pos.out_max);
        Cascade_Reset(&a->cascade, a->motor->angle_rad, a->motor->speed_rad_s);
        Traj_Start(&a->traj, s_pair_from[i], s_pair_from[i] + s_pair_direction[i] * s_pair_traj.target);
        a->state = AXIS_MOVING;
        a->settled_s = 0.0f;
    }
    return MOTORAPP_OK;
}

static float pair_limit(float value, float limit)
{
    if (value > limit) { return limit; }
    if (value < -limit) { return -limit; }
    return value;
}

/* 只写观察数据；不参与控制或到位判定。上一采样阶段累计本次HAL间隔。 */
static void pair_record_timing(uint32_t now, float previous_ref_speed)
{
    MotorApp_2006Timing_t *t = &s_pair_status.timing;
    uint32_t delta;
    uint8_t i;
    float target, direction, ref_speed;
    if (t->status != 1u) { return; }
    delta = now - s_pair_timing_last_tick;
    s_pair_timing_last_tick = now;
    t->total_ms = now - t->start_ms;
    switch (t->phase) {
    case 1u: t->accel_ms += delta; break;
    case 2u: t->cruise_ms += delta; break;
    case 3u: t->decel_ms += delta; break;
    case 4u: t->follow_ms += delta; break;
    default: break;
    }
    target = s_pair_traj.target * s_axis[3].motor->model->gear_ratio / TWO_PI;
    direction = target < 0.0f ? -1.0f : 1.0f;
    for (i = 0u; i < 2u; i++) {
        float actual = s_pair_status.actual_turns[i];
        float error = fabsf(target - actual);
        if ((t->first_motion_ms[i] == UINT32_MAX) && (fabsf(actual) >= 0.01f)) {
            t->first_motion_ms[i] = t->total_ms;
        }
        if (error <= MOTORAPP_POSITION_TOLERANCE_ROTOR_TURNS) {
            if (t->first_in_tol_ms[i] == UINT32_MAX) { t->first_in_tol_ms[i] = t->total_ms; }
        } else if (fabsf((float)s_axis[s_pair_axis[i]].motor->raw_speed) <= 10.0f) {
            t->stopped_ms[i] += delta;
        }
    }
    if ((t->reach_75_ms == UINT32_MAX) &&
        (s_pair_status.actual_turns[0] * direction >= fabsf(target) * 0.75f) &&
        (s_pair_status.actual_turns[1] * direction >= fabsf(target) * 0.75f)) {
        t->reach_75_ms = t->total_ms;
    }
    if (s_pair_status.state == MOTORAPP_PAIR_FAULT) {
        t->phase = 6u;
        t->status = 3u;
        return;
    }
    if ((t->planner_ms == UINT32_MAX) && (s_pair_traj.active == 0u)) {
        t->planner_ms = t->total_ms;
    }
    if (s_pair_status.state == MOTORAPP_PAIR_DONE) {
        t->phase = 5u;
        t->status = 2u;
    } else if (s_pair_traj.active == 0u) {
        t->phase = 4u;
    } else {
        ref_speed = fabsf(s_pair_traj.vel_ref);
        if (ref_speed > previous_ref_speed + 0.000001f) { t->phase = 1u; }
        else if (ref_speed < previous_ref_speed - 0.000001f) { t->phase = 3u; }
        else { t->phase = 2u; }
    }
}

static void pair_fault(MotorApp_PairFault_t reason)
{
    uint8_t i;
    s_pair_status.state = MOTORAPP_PAIR_FAULT;
    s_pair_status.fault = reason;
    s_pair_status.fault_tick_ms = HAL_GetTick();
    pair_record_timing(s_pair_status.fault_tick_ms, fabsf(s_pair_traj.vel_ref));
    s_pair_status.time_scale = 0.0f;
    s_pair_status.settled_ms = 0.0f;
    Traj_Stop(&s_pair_traj);
    for (i = 0u; i < 2u; i++) {
        Axis_t *a = &s_axis[s_pair_axis[i]];
        uint32_t last_rx_tick = a->motor->rx_tick;
        s_pair_status.fault_rx_age_ms[i] = HAL_GetTick() - last_rx_tick;
        s_pair_status.fault_rx_count[i] = a->motor->rx_count;
        s_pair_status.fault_command[i] = a->motor->cmd;
        Traj_Stop(&a->traj);
        Cascade_Reset(&a->cascade, a->motor->angle_rad, a->motor->speed_rad_s);
        a->motor->cmd = 0;
        a->state = AXIS_FAULT;
        a->settled_s = 0.0f;
        s_pair_speed[i] = 0.0f;
        s_pair_status.speed_command_rpm[i] = 0.0f;
    }
}

/* 双轮组拥有运动、到位和保持全过程；不再调用两次 axis_update。 */
static void pair_update(float dt)
{
    float q[2], ratio, rotor_scale, correction;
    float previous_ref_speed = fabsf(s_pair_traj.vel_ref);
    float motion_direction = s_pair_traj.target < 0.0f ? -1.0f : 1.0f;
    uint8_t i;
    const MotorPair_Cfg_t *cfg = &MOTORPAIR_CFG_2006;
    if (s_pair_status.state == MOTORAPP_PAIR_FAULT) { return; }
    ratio = s_axis[3].motor->model->gear_ratio;
    rotor_scale = ratio / TWO_PI;
    for (i = 0u; i < 2u; i++) {
        Axis_t *a = &s_axis[s_pair_axis[i]];
        if (DjiMotor_IsOnline(a->motor) == 0) {
            s_pair_status.fault_motor_id = a->motor->id;
            pair_fault(MOTORAPP_PAIR_FAULT_OFFLINE);
            return;
        }
        q[i] = s_pair_direction[i] * (a->motor->angle_rad - s_pair_from[i]);
        s_pair_status.actual_turns[i] = q[i] * rotor_scale;
    }
    s_pair_status.sync_error_turns = (q[0] - q[1]) * rotor_scale;
    /* 任一侧到位就结束本段：分别锁住真实当前位置，不改编码器反馈。 */
    if ((s_pair_status.state != MOTORAPP_PAIR_DONE) &&
        ((fabsf(s_pair_traj.target - q[0]) * rotor_scale <= MOTORAPP_POSITION_TOLERANCE_ROTOR_TURNS) ||
         (fabsf(s_pair_traj.target - q[1]) * rotor_scale <= MOTORAPP_POSITION_TOLERANCE_ROTOR_TURNS))) {
        s_pair_status.timing.finish_motor_id =
            fabsf(s_pair_traj.target - q[0]) * rotor_scale <= MOTORAPP_POSITION_TOLERANCE_ROTOR_TURNS ?
            s_axis[s_pair_axis[0]].motor->id : s_axis[s_pair_axis[1]].motor->id;
        s_pair_traj.pos_ref = s_pair_traj.target;
        s_pair_traj.vel_ref = 0.0f;
        s_pair_traj.active = 0u;
        s_pair_status.state = MOTORAPP_PAIR_DONE;
        for (i = 0u; i < 2u; i++) {
            Axis_t *a = &s_axis[s_pair_axis[i]];
            a->traj.target = a->motor->angle_rad;
            a->traj.pos_ref = a->traj.target;
            a->traj.vel_ref = 0.0f;
            a->traj.active = 0u;
            Cascade_Reset(&a->cascade, a->motor->angle_rad, a->motor->speed_rad_s);
            s_pair_speed[i] = 0.0f;
        }
    }
    /* 正常时钟推进，不再因参考领先实际位置而降速或暂停。 */
    s_pair_status.time_scale = 1.0f;
    Traj_Update(&s_pair_traj, dt);
    s_pair_status.reference_turns = s_pair_traj.pos_ref * rotor_scale;
    correction = s_pair_status.state == MOTORAPP_PAIR_DONE ? 0.0f :
                 pair_limit(cfg->sync_kp * (q[0] - q[1]), cfg->sync_speed_max);
    s_pair_status.correction_rad_s = correction;
    for (i = 0u; i < 2u; i++) {
        Axis_t *a = &s_axis[s_pair_axis[i]];
        float pos_speed, common_speed, motor_speed, speed_limit = a->cascade.pos.out_max;
        a->traj.pos_ref = s_pair_status.state == MOTORAPP_PAIR_DONE ? a->traj.target :
                          s_pair_from[i] + s_pair_direction[i] * s_pair_traj.pos_ref;
        a->traj.vel_ref = s_pair_direction[i] * s_pair_traj.vel_ref;
        a->traj.active = s_pair_traj.active;
        a->cascade.pos.setpoint = a->traj.pos_ref;
        pos_speed = PID_Update_Continuous(&a->cascade.pos, a->motor->angle_rad, dt);
        common_speed = s_pair_direction[i] * pos_speed + s_pair_traj.vel_ref;
        common_speed += i == 0u ? -correction : correction;
        if (s_pair_traj.active != 0u) {
            /* 行进阶段领先侧可减速到零，不因同步纠偏直接掉头。 */
            if (common_speed * motion_direction < 0.0f) { common_speed = 0.0f; }
        }
        motor_speed = s_pair_direction[i] * pair_limit(common_speed, speed_limit);
        s_pair_speed[i] += pair_limit(motor_speed - s_pair_speed[i], cfg->command_a_max * dt);
        s_pair_speed[i] = pair_limit(s_pair_speed[i], a->cascade.pos.out_max);
        a->cascade.spd.setpoint = s_pair_speed[i];
        a->motor->cmd = (int16_t)PID_Update_Continuous(&a->cascade.spd, a->motor->speed_rad_s, dt);
        s_pair_status.speed_command_rpm[i] = s_pair_speed[i] * ratio * 60.0f / TWO_PI;
    }
    s_pair_status.settled_ms = 0.0f;
    for (i = 0u; i < 2u; i++) {
        Axis_t *a = &s_axis[s_pair_axis[i]];
        a->settled_s = 0.0f;
        a->state = s_pair_status.state == MOTORAPP_PAIR_DONE ? AXIS_IDLE : AXIS_MOVING;
    }
    pair_record_timing(HAL_GetTick(), previous_ref_speed);
}

/* ============================== 单轴控制 ============================== */

static void axis_update(Axis_t *ax, float dt, uint8_t sync_reference)
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
        ax->settled_s = 0.0f;
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
    if (sync_reference == 0u) { Traj_Update(&ax->traj, dt); }

    /* 位置环 → 速度环 → 电流命令。
     * pos_ref 是【规划过的】参考位置，不是目标位置 —— 目标已经由规划器
     * 铺成了一条速度受限的曲线，误差才不会被撑大。 */
    ax->motor->cmd = (int16_t)Cascade_Update(&ax->cascade,
                                             ax->motor->angle_rad,
                                             ax->motor->speed_rad_s,
                                             ax->traj.pos_ref,
                                             ax->traj.vel_ref,
                                             dt);

    if (ax->state == AXIS_MOVING) {
        float ratio = ax->motor->model->gear_ratio;
        float pos_tol = MOTORAPP_POSITION_TOLERANCE_ROTOR_TURNS * TWO_PI / ratio;
        float speed_tol = MOTORAPP_SPEED_TOLERANCE_ROTOR_RPM * TWO_PI / (60.0f * ratio);
        if ((Traj_IsActive(&ax->traj) == 0) &&
            (fabsf(ax->traj.target - ax->motor->angle_rad) <= pos_tol) &&
            (fabsf(ax->motor->speed_rad_s) <= speed_tol)) {
            ax->settled_s += dt;
            if (ax->settled_s >= MOTORAPP_SETTLE_TIME_S) { ax->state = AXIS_IDLE; }
        } else {
            ax->settled_s = 0.0f;
        }
    }
}

/* ============================== 组帧发送 ============================== */

/* 按总线去重后，每条总线各发自己的 0x200 / 0x1FF / 0x2FF。
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
        (void)DjiMotor_SendGroup(bus[i], 1u);   /* 0x1FF，3508/2006 id5~8、6020 id1~4 */
        (void)DjiMotor_SendGroup(bus[i], 2u);   /* 0x2FF，GM6020 id 5~7 */
    }
}

/* ============================== 主循环 ============================== */

void MotorApp_Update(float dt)
{
    uint8_t i;
    uint8_t sync_reference = s_sync_running;
    uint8_t sync_fault = 0u;

    /* 1) 先把所有轴的反馈换算完。
     *    必须排在控制之前，而且要全部做完再开始控 —— 这样每根轴用的都是
     *    同一拍的反馈，轴和轴之间不会差半拍。 */
    for (i = 0u; i < AXIS_COUNT; i++) {
        DjiMotor_Update(s_axis[i].motor);
    }

    if (s_sync_running != 0u) {
        for (i = 0u; i < 3u; i++) {
            if (DjiMotor_IsOnline(s_axis[i].motor) == 0) { sync_fault = 1u; }
        }
        if (sync_fault != 0u) {
            /* 任一同步轴掉线，整组放弃旧行程；在线轴随后保持当前位置。
             * 故障状态至少保留这一拍，任务才能锁住后续步骤。 */
            Traj_Stop(&s_sync_traj);
            s_sync_running = 0u;
            for (i = 0u; i < 3u; i++) {
                Traj_Start(&s_axis[i].traj, s_axis[i].motor->angle_rad, s_axis[i].motor->angle_rad);
                Traj_Stop(&s_axis[i].traj);
                Cascade_Reset(&s_axis[i].cascade, s_axis[i].motor->angle_rad, s_axis[i].motor->speed_rad_s);
                s_axis[i].settled_s = 0.0f;
                s_axis[i].state = AXIS_FAULT;
                s_axis[i].motor->cmd = 0;
            }
        } else {
            Traj_Update(&s_sync_traj, dt);
            for (i = 0u; i < 3u; i++) {
                s_axis[i].traj.pos_ref = s_sync_from[i] + s_sync_delta[i] * s_sync_traj.pos_ref;
                s_axis[i].traj.vel_ref = s_sync_delta[i] * s_sync_traj.vel_ref;
                s_axis[i].traj.active = s_sync_traj.active;
            }
        }
    }

    /* 2) 逐轴算控制量 */
    for (i = 0u; i < AXIS_COUNT; i++) {
        if ((sync_fault != 0u) && (i < 3u)) { continue; }
        if ((s_pair_status.state != MOTORAPP_PAIR_IDLE) && (i >= 3u)) { continue; }
        axis_update(&s_axis[i], dt, (uint8_t)((sync_reference != 0u) && (i < 3u)));
    }
    if (s_pair_status.state != MOTORAPP_PAIR_IDLE) { pair_update(dt); }
    if ((s_sync_running != 0u) && (Traj_IsActive(&s_sync_traj) == 0)) { s_sync_running = 0u; }

    /* 3) 统一发出。命令必须全部算完再发 —— 0x200 一帧装 4 台电机，
     *    边算边发会拆成两帧，中间的半拍状态没有意义。 */
    send_all_buses();
    
}
