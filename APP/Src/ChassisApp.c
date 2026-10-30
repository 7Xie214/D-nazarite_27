/** @brief Four independent steering cascades and drive speed loops. */
#include "ChassisApp.h"

ChassisApp_Status_t g_chassis_app_status = CHASSIS_APP_FAULT_CONFIG;
ChassisFront_State_t g_chassis_front_state = CHASSIS_FRONT_HOLD;

#if CHASSIS_ENABLE_SWERVE
#include "PID_Config.h"
#include "fdcan.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define CHASSIS_PI 3.14159265358979323846f
#define CHASSIS_TWO_PI (2.0f * CHASSIS_PI)

ChassisApp_Wheel_t g_chassis_wheels[SWERVE_MODULE_COUNT];
ChassisFront_Axis_t g_chassis_front_axes[2];
static volatile float s_vx, s_vy, s_wz;
static float s_front_settle_s;
static uint8_t s_front_ready;
static uint8_t s_front_arrived;

static int finite_value(float value)
{
    return (value >= -FLT_MAX) && (value <= FLT_MAX);
}

/* CAN3 belongs to MotorApp. Empty physical groups produce no actual frame. */
static void send_chassis(void)
{
    uint8_t group;
    for (group = 0u; group < 3u; group++) {
        (void)DjiMotor_SendGroup(&hfdcan1, group);
        (void)DjiMotor_SendGroup(&hfdcan2, group);
    }
}

static void reset_controllers(ChassisApp_Wheel_t *wheel)
{
    float angle = wheel->module->actual_angle_rad;
    float steer_speed = wheel->steer_speed_rad_s;
    float drive_speed = wheel->drive_speed_mps;
    Cascade_Reset(&wheel->steer_pid,
                  finite_value(angle) ? angle : 0.0f,
                  finite_value(steer_speed) ? steer_speed : 0.0f);
    wheel->drive_pid.setpoint = 0.0f;
    wheel->drive_pid.integral = 0.0f;
    wheel->drive_pid.d_filt = 0.0f;
    wheel->drive_pid.prev_actual = finite_value(drive_speed) ? drive_speed : 0.0f;
}

static void stop_chassis(ChassisApp_Status_t status)
{
    uint8_t i;
    s_front_ready = 0u;
    s_front_settle_s = 0.0f;
    s_front_arrived = 0u;
    if (g_chassis_front_state == CHASSIS_FRONT_MOVING) {
        g_chassis_front_state = CHASSIS_FRONT_FAULT;
    }
    for (i = 0u; i < 2u; i++) {
        ChassisFront_Axis_t *axis = &g_chassis_front_axes[i];
        Traj_Stop(&axis->traj);
        axis->seeded = 0u;
        Cascade_Reset(&axis->cascade, 0.0f, 0.0f);
    }
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        ChassisApp_Wheel_t *wheel = &g_chassis_wheels[i];
        wheel->drive_motor->cmd = 0;
        wheel->steer_motor->cmd = 0;
        wheel->module->target_speed_mps = 0.0f;
        wheel->feedback_ready = 0u;
        reset_controllers(wheel);
    }
    g_chassis_app_status = status;
    send_chassis();
}

static int configuration_valid(int front_mode)
{
    uint8_t i;
    if (!front_mode && (!finite_value(CHASSIS_SWERVE_MAX_WHEEL_SPEED_MPS) ||
        (CHASSIS_SWERVE_MAX_WHEEL_SPEED_MPS <= 0.0f))) {
        return 0;
    }
    if (front_mode &&
        (!finite_value(CHASSIS_FRONT_MOVE_POS_TOL_RAD) ||
         (CHASSIS_FRONT_MOVE_POS_TOL_RAD < 0.0f) ||
         !finite_value(CHASSIS_FRONT_MOVE_SPEED_TOL_RAD_S) ||
         (CHASSIS_FRONT_MOVE_SPEED_TOL_RAD_S < 0.0f) ||
         !finite_value(CHASSIS_FRONT_MOVE_SETTLE_S) ||
         (CHASSIS_FRONT_MOVE_SETTLE_S <= 0.0f))) { return 0; }
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        const Swerve_Module_t *module = g_chassis_wheels[i].module;
        const rudder_direction_t *steer = &module->rudder_direction;
        const steering_gear_t *drive = &module->steering_gear;
        if ((!front_mode && (!finite_value(module->half_wheel_base) ||
            !finite_value(module->half_track_width) || !finite_value(drive->R) ||
            (module->half_wheel_base <= 0.0f) ||
            (module->half_track_width <= 0.0f) || (drive->R <= 0.0f))) ||
            (steer->zero_offset >= 8192u) ||
            ((steer->encoder_direction != 1) && (steer->encoder_direction != -1)) ||
            ((steer->electirc_direction != 1) && (steer->electirc_direction != -1)) ||
            ((drive->encoder_direction != 1) && (drive->encoder_direction != -1)) ||
            ((drive->electric_direction != 1) && (drive->electric_direction != -1)) ||
            ((module->angle_range != SWERVE_ANGLE_MINUS_90_TO_90) &&
             (module->angle_range != SWERVE_ANGLE_0_TO_180) &&
             (module->angle_range != SWERVE_ANGLE_CONTINUOUS))) {
            return 0;
        }
    }
    return 1;
}

/* Absolute feedback defines the mechanical zero; accumulated feedback provides
 * continuous turns only after a valid absolute seed. Never clamp a limit breach. */
static int read_feedback(ChassisApp_Wheel_t *wheel, int front_mode)
{
    Swerve_Module_t *module = wheel->module;
    const DjiMotor_t *motor = wheel->steer_motor;
    float angle;
    float signed_motor_angle;
    int32_t counts;

    if ((wheel->drive_motor->model == NULL) || (motor->model == NULL) ||
        (DjiMotor_IsOnline(wheel->drive_motor) == 0) ||
        (DjiMotor_IsOnline(motor) == 0) ||
        (motor->prev_raw_angle < 0) || (motor->prev_raw_angle >= 8192)) {
        return 0;
    }
    if (!finite_value(wheel->drive_motor->angle_rad) ||
        !finite_value(wheel->drive_motor->speed_rad_s) ||
        !finite_value(wheel->drive_motor->model->gear_ratio) ||
        (wheel->drive_motor->model->gear_ratio <= 0.0f)) { return 0; }
    wheel->drive_speed_mps = front_mode ? 0.0f :
        (float)module->steering_gear.encoder_direction *
        wheel->drive_motor->speed_rad_s * module->steering_gear.R;
    wheel->steer_speed_rad_s = (float)module->rudder_direction.encoder_direction *
                              motor->speed_rad_s;
    signed_motor_angle = (float)module->rudder_direction.encoder_direction * motor->angle_rad;
    if (!finite_value(wheel->drive_speed_mps) ||
        !finite_value(wheel->steer_speed_rad_s) || !finite_value(signed_motor_angle)) {
        return 0;
    }

    /* Use the sample consumed by DjiMotor_Update, matching angle_rad. A CAN IRQ
     * may already have replaced raw_angle with the next frame. */
    counts = (motor->prev_raw_angle - (int32_t)module->rudder_direction.zero_offset) *
             (int32_t)module->rudder_direction.encoder_direction;
    counts %= 8192;
    if (counts < 0) { counts += 8192; }
    if ((module->angle_range != SWERVE_ANGLE_0_TO_180) && (counts > 4096)) {
        counts -= 8192;
    }
    /* Check exact encoder limits before float conversion. This also keeps the
     * legal endpoint independent of a compiler's intermediate FP precision. */
    if ((module->angle_range == SWERVE_ANGLE_MINUS_90_TO_90) &&
        ((counts < -2048) || (counts > 2048))) { return 0; }
    if ((module->angle_range == SWERVE_ANGLE_0_TO_180) && (counts > 4096)) {
        return 0;
    }
    angle = (float)counts * (CHASSIS_TWO_PI / DJI_COUNTS_PER_REV);

    if (module->angle_range == SWERVE_ANGLE_CONTINUOUS) {
        if (wheel->feedback_ready == 0u) {
            if (wheel->angle_seeded != 0u) {
                /* A single-turn encoder cannot observe turns made offline. */
                angle += floorf((module->actual_angle_rad - angle) / CHASSIS_TWO_PI + 0.5f) *
                         CHASSIS_TWO_PI;
            }
            wheel->steer_angle_bias_rad = angle - signed_motor_angle;
        } else {
            angle = wheel->steer_angle_bias_rad + signed_motor_angle;
        }
    }
    if (!finite_value(angle)) { return 0; }

    module->actual_angle_rad = angle;
    wheel->angle_seeded = 1u;
    if (wheel->feedback_ready == 0u) {
        module->target_angle_rad = angle;
        module->target_speed_mps = 0.0f;
        reset_controllers(wheel);
        wheel->feedback_ready = 1u;
    }
    return 1;
}

static int16_t raw_command(float value, int8_t direction, int16_t limit)
{
    value *= (float)direction;
    if (value > (float)limit) { value = (float)limit; }
    if (value < -(float)limit) { value = -(float)limit; }
    return (int16_t)value;
}

void ChassisApp_Init(void)
{
    static DjiMotor_t *const drive[SWERVE_MODULE_COUNT] = {
        &g_motor_3508_4, &g_motor_3508_5, &g_motor_3508_6, &g_motor_3508_7
    };
    static DjiMotor_t *const steer[SWERVE_MODULE_COUNT] = {
        &g_motor_6020_1, &g_motor_6020_4, &g_motor_6020_2, &g_motor_6020_3
    };
    uint8_t i;
    memset(g_chassis_wheels, 0, sizeof(g_chassis_wheels));
    memset(g_chassis_front_axes, 0, sizeof(g_chassis_front_axes));
    g_chassis_front_state = CHASSIS_FRONT_HOLD;
    s_front_settle_s = 0.0f;
    s_front_arrived = 0u;
    s_front_ready = 0u;
    ChassisApp_SetVelocity(0.0f, 0.0f, 0.0f);
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        ChassisApp_Wheel_t *wheel = &g_chassis_wheels[i];
        const Chassis_DrivePID_Cfg_t *cfg = &CHASSIS_PID_CFG[i].drive;
        wheel->module = &g_swerve_modules[i];
        wheel->drive_motor = drive[i];
        wheel->steer_motor = steer[i];
        Cascade_Init(&wheel->steer_pid, &CHASSIS_PID_CFG[i].steer);
        PID_Init(&wheel->drive_pid, cfg->kp, cfg->ki, cfg->kd,
                 cfg->integral_max, cfg->out_max, cfg->d_alpha,
                 cfg->anti_windup, cfg->integ_sep_thr);
        wheel->drive_motor->cmd = 0;
        wheel->steer_motor->cmd = 0;
    }
    for (i = 0u; i < 2u; i++) {
        g_chassis_front_axes[i].motor = drive[i];
        PID_Config_Apply(&g_chassis_front_axes[i].cascade,
                         &g_chassis_front_axes[i].traj, &CHASSIS_FRONT_MOVE_CFG[i]);
    }
    g_chassis_app_status = CHASSIS_APP_FAULT_FEEDBACK;
}

void ChassisApp_SetVelocity(float vx, float vy, float wz)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    s_vx = vx;
    s_vy = vy;
    s_wz = wz;
    __set_PRIMASK(mask);
}

static int front_motion_configuration_valid(void)
{
    uint8_t i;
    for (i = 0u; i < 2u; i++) {
        const ChassisFront_Axis_t *axis = &g_chassis_front_axes[i];
        if (!finite_value(axis->traj.v_max) || (axis->traj.v_max <= 0.0f) ||
            !finite_value(axis->traj.a_max) || (axis->traj.a_max <= 0.0f) ||
            !finite_value(axis->cascade.pos.out_max) ||
            (axis->cascade.pos.out_max <= 0.0f)) { return 0; }
    }
    return 1;
}

ChassisFront_StartResult_t ChassisApp_MoveFrontTurns(float left_rotor_turns,
                                                   float right_rotor_turns)
{
    float turns[2], from[2], target[2];
    uint8_t i;
    if (g_chassis_front_state == CHASSIS_FRONT_MOVING) {
        return CHASSIS_FRONT_START_BUSY;
    }
    if (!finite_value(left_rotor_turns) || !finite_value(right_rotor_turns) ||
        ((left_rotor_turns == 0.0f) && (right_rotor_turns == 0.0f))) {
        return CHASSIS_FRONT_START_BAD_CONFIG;
    }
    if ((s_front_ready == 0u) || (g_chassis_app_status != CHASSIS_APP_READY)) {
        return CHASSIS_FRONT_START_NOT_READY;
    }
    if (!configuration_valid(1) || !front_motion_configuration_valid()) {
        return CHASSIS_FRONT_START_BAD_CONFIG;
    }
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        const ChassisApp_Wheel_t *wheel = &g_chassis_wheels[i];
        if ((wheel->feedback_ready == 0u) ||
            !DjiMotor_IsOnline(wheel->drive_motor) ||
            !DjiMotor_IsOnline(wheel->steer_motor)) {
            return CHASSIS_FRONT_START_NOT_READY;
        }
    }
    turns[0] = left_rotor_turns;
    turns[1] = right_rotor_turns;
    /* Validate both before changing either planner. DjiMotor feedback is already
     * gearbox-output rad; only the requested rotor travel needs division. */
    for (i = 0u; i < 2u; i++) {
        const ChassisFront_Axis_t *axis = &g_chassis_front_axes[i];
        if ((axis->seeded == 0u) || (axis->motor->model == NULL)) {
            return CHASSIS_FRONT_START_NOT_READY;
        }
        if (!finite_value(axis->motor->model->gear_ratio) ||
            (axis->motor->model->gear_ratio <= 0.0f)) {
            return CHASSIS_FRONT_START_BAD_CONFIG;
        }
        from[i] = (float)g_chassis_wheels[i].module->steering_gear.encoder_direction *
                  axis->motor->angle_rad;
        target[i] = from[i] + turns[i] *
                    (CHASSIS_TWO_PI / axis->motor->model->gear_ratio);
        if (!finite_value(from[i]) || !finite_value(target[i])) {
            return CHASSIS_FRONT_START_BAD_CONFIG;
        }
    }
    for (i = 0u; i < 2u; i++) {
        Traj_Start(&g_chassis_front_axes[i].traj, from[i], target[i]);
    }
    s_front_settle_s = 0.0f;
    s_front_arrived = 0u;
    g_chassis_front_state = CHASSIS_FRONT_MOVING;
    return CHASSIS_FRONT_START_OK;
}

static ChassisApp_Status_t update_front_controllers(float dt)
{
    uint8_t i;
    int arrived = 1;
    if ((g_chassis_front_state == CHASSIS_FRONT_MOVING) &&
        !front_motion_configuration_valid()) { return CHASSIS_APP_FAULT_CONFIG; }
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        ChassisApp_Wheel_t *wheel = &g_chassis_wheels[i];
        Swerve_Module_t *module = wheel->module;
        float steer_output, drive_output = 0.0f;
        float target_angle = module->target_angle_rad;
        if (!finite_value(target_angle) ||
            ((module->angle_range == SWERVE_ANGLE_MINUS_90_TO_90) &&
             ((target_angle < -CHASSIS_PI / 2.0f) || (target_angle > CHASSIS_PI / 2.0f))) ||
            ((module->angle_range == SWERVE_ANGLE_0_TO_180) &&
             ((target_angle < 0.0f) || (target_angle > CHASSIS_PI)))) {
            return CHASSIS_APP_FAULT_CONFIG;
        }
        module->target_speed_mps = 0.0f;
        if (i < 2u) {
            ChassisFront_Axis_t *axis = &g_chassis_front_axes[i];
            float direction = (float)module->steering_gear.encoder_direction;
            float position = direction * axis->motor->angle_rad;
            float speed = direction * axis->motor->speed_rad_s;
            if (axis->seeded == 0u) {
                Traj_Start(&axis->traj, position, position);
                Traj_Stop(&axis->traj);
                Cascade_Reset(&axis->cascade, position, speed);
                axis->seeded = 1u;
            }
            Traj_Update(&axis->traj, dt);
            drive_output = Cascade_Update(&axis->cascade, position, speed,
                                          axis->traj.pos_ref, axis->traj.vel_ref, dt);
            if (Traj_IsActive(&axis->traj) ||
                (fabsf(axis->traj.target - position) > CHASSIS_FRONT_MOVE_POS_TOL_RAD) ||
                (fabsf(speed) > CHASSIS_FRONT_MOVE_SPEED_TOL_RAD_S)) { arrived = 0; }
        }
        steer_output = Cascade_Update(&wheel->steer_pid, module->actual_angle_rad,
                                      wheel->steer_speed_rad_s, target_angle, 0.0f, dt);
        if (!finite_value(drive_output) || !finite_value(steer_output)) {
            return CHASSIS_APP_FAULT_OUTPUT;
        }
        wheel->drive_motor->cmd = raw_command(drive_output,
            module->steering_gear.electric_direction, wheel->drive_motor->model->cmd_max);
        wheel->steer_motor->cmd = raw_command(steer_output,
            module->rudder_direction.electirc_direction, wheel->steer_motor->model->cmd_max);
    }
    if (g_chassis_front_state == CHASSIS_FRONT_MOVING) {
        if (!arrived) {
            s_front_settle_s = 0.0f;
            s_front_arrived = 0u;
        } else if (s_front_arrived == 0u) {
            /* The first qualifying sample begins the interval. Its preceding
             * dt may include travel, so only later qualifying samples add time. */
            s_front_arrived = 1u;
            s_front_settle_s = 0.0f;
        } else if (dt >= CHASSIS_FRONT_MOVE_SETTLE_S - s_front_settle_s) {
            g_chassis_front_state = CHASSIS_FRONT_DONE;
        } else {
            s_front_settle_s += dt;
        }
    }
    return CHASSIS_APP_READY;
}

static void update_chassis(float dt, int front_mode)
{
    float vx, vy, wz;
    uint32_t mask;
    uint8_t i;
    int feedback_valid = 1;

    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        /* A first frame arriving after Update must wait until the next cycle;
         * its live rx_count does not make the consumed startup zero valid. */
        if ((DjiMotor_IsOnline(g_chassis_wheels[i].drive_motor) == 0) ||
            (DjiMotor_IsOnline(g_chassis_wheels[i].steer_motor) == 0)) {
            feedback_valid = 0;
        }
        DjiMotor_Update(g_chassis_wheels[i].drive_motor);
        DjiMotor_Update(g_chassis_wheels[i].steer_motor);
    }
    if (!finite_value(dt) || (dt <= 0.0f)) {
        stop_chassis(CHASSIS_APP_FAULT_DT);
        return;
    }
    if (!configuration_valid(front_mode)) {
        stop_chassis(CHASSIS_APP_FAULT_CONFIG);
        return;
    }
    if (!feedback_valid) {
        stop_chassis(CHASSIS_APP_FAULT_FEEDBACK);
        return;
    }
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        if (!read_feedback(&g_chassis_wheels[i], front_mode)) { feedback_valid = 0; }
    }
    if (!feedback_valid) {
        stop_chassis(CHASSIS_APP_FAULT_FEEDBACK);
        return;
    }

    if (front_mode) {
        ChassisApp_Status_t status = update_front_controllers(dt);
        if (status != CHASSIS_APP_READY) {
            stop_chassis(status);
            return;
        }
        s_front_ready = 1u;
        g_chassis_app_status = CHASSIS_APP_READY;
        send_chassis();
        return;
    }
    s_front_ready = 0u;
    for (i = 0u; i < 2u; i++) { g_chassis_front_axes[i].seeded = 0u; }

    mask = __get_PRIMASK();
    __disable_irq();
    vx = s_vx;
    vy = s_vy;
    wz = s_wz;
    __set_PRIMASK(mask);
    if (Swerve_Calculate(g_swerve_modules, SWERVE_MODULE_COUNT, vx, vy, wz,
                         CHASSIS_SWERVE_MAX_WHEEL_SPEED_MPS) != SWERVE_OK) {
        stop_chassis(CHASSIS_APP_FAULT_SOLVE);
        return;
    }
    for (i = 0u; i < SWERVE_MODULE_COUNT; i++) {
        ChassisApp_Wheel_t *wheel = &g_chassis_wheels[i];
        const Swerve_Module_t *module = wheel->module;
        float steer_output, drive_output;
        wheel->drive_pid.setpoint = module->target_speed_mps;
        drive_output = PID_Update_Continuous(&wheel->drive_pid, wheel->drive_speed_mps, dt);
        steer_output = Cascade_Update(&wheel->steer_pid, module->actual_angle_rad,
                                      wheel->steer_speed_rad_s, module->target_angle_rad,
                                      0.0f, dt);
        if (!finite_value(drive_output) || !finite_value(steer_output)) {
            stop_chassis(CHASSIS_APP_FAULT_OUTPUT);
            return;
        }
        wheel->drive_motor->cmd = raw_command(drive_output,
            module->steering_gear.electric_direction, wheel->drive_motor->model->cmd_max);
        wheel->steer_motor->cmd = raw_command(steer_output,
            module->rudder_direction.electirc_direction, wheel->steer_motor->model->cmd_max);
    }
    g_chassis_app_status = CHASSIS_APP_READY;
    send_chassis();
}

void ChassisApp_Update(float dt) { update_chassis(dt, 0); }
void ChassisApp_UpdateFrontMove(float dt) { update_chassis(dt, 1); }

#else

/* Reserved wheel systems have no motor control/task ownership yet. */
void ChassisApp_Init(void)
{
    g_chassis_app_status = CHASSIS_APP_FAULT_CONFIG;
    g_chassis_front_state = CHASSIS_FRONT_HOLD;
}
void ChassisApp_SetVelocity(float vx, float vy, float wz)
{
    (void)vx; (void)vy; (void)wz;
}
void ChassisApp_Update(float dt) { (void)dt; }
void ChassisApp_UpdateFrontMove(float dt) { (void)dt; }
ChassisFront_StartResult_t ChassisApp_MoveFrontTurns(float left_rotor_turns,
                                                   float right_rotor_turns)
{
    (void)left_rotor_turns; (void)right_rotor_turns;
    return CHASSIS_FRONT_START_BAD_CONFIG;
}

#endif /* CHASSIS_ENABLE_SWERVE */
