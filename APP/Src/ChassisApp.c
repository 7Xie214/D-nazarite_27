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
ChassisFrontPair_Status_t g_chassis_front_pair;
static Traj_t s_front_pair_traj;
static float s_front_pair_from[2];
static float s_front_pair_direction[2];
static float s_front_pair_turns[2];
static float s_front_pair_speed[2];
static ChassisApp_Status_t s_front_pair_fault_status;
static uint32_t s_front_pair_sequence;
static float s_front_pair_full_distance;
static uint8_t s_front_pair_pause_armed;
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
static int read_steer_feedback(ChassisApp_Wheel_t *wheel)
{
    Swerve_Module_t *module = wheel->module;
    const DjiMotor_t *motor = wheel->steer_motor;
    float angle;
    float signed_motor_angle;
    int32_t counts;

    if ((motor->model == NULL) || (DjiMotor_IsOnline(motor) == 0) ||
        (motor->prev_raw_angle < 0) || (motor->prev_raw_angle >= 8192)) {
        return 0;
    }
    wheel->steer_speed_rad_s = (float)module->rudder_direction.encoder_direction *
                              motor->speed_rad_s;
    signed_motor_angle = (float)module->rudder_direction.encoder_direction * motor->angle_rad;
    if (!finite_value(wheel->steer_speed_rad_s) || !finite_value(signed_motor_angle)) {
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

static int read_feedback(ChassisApp_Wheel_t *wheel, int front_mode)
{
    const DjiMotor_t *drive = wheel->drive_motor;
    if ((drive->model == NULL) || !DjiMotor_IsOnline(drive) ||
        !finite_value(drive->angle_rad) || !finite_value(drive->speed_rad_s) ||
        !finite_value(drive->model->gear_ratio) || (drive->model->gear_ratio <= 0.0f)) {
        return 0;
    }
    wheel->drive_speed_mps = front_mode ? 0.0f :
        (float)wheel->module->steering_gear.encoder_direction *
        drive->speed_rad_s * wheel->module->steering_gear.R;
    return finite_value(wheel->drive_speed_mps) && read_steer_feedback(wheel);
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
    memset(&g_chassis_front_pair, 0, sizeof(g_chassis_front_pair));
    memset(s_front_pair_speed, 0, sizeof(s_front_pair_speed));
    s_front_pair_fault_status = CHASSIS_APP_READY;
    s_front_pair_sequence = 0u;
    s_front_pair_pause_armed = 0u;
    Traj_Init(&s_front_pair_traj, MOTORPAIR_CFG_CHASSIS_FRONT.traj_v_max,
              MOTORPAIR_CFG_CHASSIS_FRONT.traj_a_max);
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

/* Return front-steering status separately: rear faults must not block the
 * existing front-drive pair. The public steering-only status covers all used modules. */
static ChassisApp_Status_t update_steer_zero_hold(float dt, uint8_t count)
{
    uint8_t i;
    ChassisApp_Status_t status = CHASSIS_APP_READY;
    ChassisApp_Status_t front_status = CHASSIS_APP_READY;

    for (i = 0u; i < count; i++) {
        ChassisApp_Wheel_t *wheel = &g_chassis_wheels[i];
        Swerve_Module_t *module = wheel->module;
        const rudder_direction_t *steer = &module->rudder_direction;
        ChassisApp_Status_t fault = CHASSIS_APP_READY;
        float output = 0.0f;

        module->target_angle_rad = 0.0f;
        module->target_speed_mps = 0.0f;
        if (!finite_value(dt) || (dt <= 0.0f)) {
            fault = CHASSIS_APP_FAULT_DT;
        } else if ((steer->zero_offset >= 8192u) ||
                   ((steer->encoder_direction != 1) && (steer->encoder_direction != -1)) ||
                   ((steer->electirc_direction != 1) && (steer->electirc_direction != -1)) ||
                   ((module->angle_range != SWERVE_ANGLE_MINUS_90_TO_90) &&
                    (module->angle_range != SWERVE_ANGLE_0_TO_180) &&
                    (module->angle_range != SWERVE_ANGLE_CONTINUOUS))) {
            fault = CHASSIS_APP_FAULT_CONFIG;
        } else if (!DjiMotor_IsOnline(wheel->steer_motor)) {
            fault = CHASSIS_APP_FAULT_FEEDBACK;
        } else {
            /* Do not seed the accumulated encoder before its first real frame. */
            DjiMotor_Update(wheel->steer_motor);
            if (!read_steer_feedback(wheel)) {
                fault = CHASSIS_APP_FAULT_FEEDBACK;
            } else {
                /* Feedback seeding normally holds the measured angle. This mode
                 * always requests the calibrated mechanical zero instead. */
                module->target_angle_rad = 0.0f;
                output = Cascade_Update(&wheel->steer_pid, module->actual_angle_rad,
                                        wheel->steer_speed_rad_s, 0.0f, 0.0f, dt);
                if (!finite_value(output)) { fault = CHASSIS_APP_FAULT_OUTPUT; }
            }
        }
        if (fault != CHASSIS_APP_READY) {
            output = 0.0f;
            wheel->feedback_ready = 0u;
            reset_controllers(wheel);
            if (status == CHASSIS_APP_READY) { status = fault; }
            if ((i < 2u) && (front_status == CHASSIS_APP_READY)) { front_status = fault; }
        }
        wheel->steer_motor->cmd = raw_command(output, steer->electirc_direction,
            wheel->steer_motor->model != NULL ? wheel->steer_motor->model->cmd_max : 0);
    }
    g_chassis_app_status = status;
    for (i = count; i < SWERVE_MODULE_COUNT; i++) {
        g_chassis_wheels[i].steer_motor->cmd = 0;
    }
    /* One 0x1ff frame per bus carries front and rear steering slots together.
     * Drive commands and CAN3 belong to their separate control paths. */
    (void)DjiMotor_SendGroup(&hfdcan1, 1u);
    (void)DjiMotor_SendGroup(&hfdcan2, 1u);
    return front_status;
}

void ChassisApp_UpdateFrontSteerHold(float dt) { (void)update_steer_zero_hold(dt, 2u); }
/* 顺序FL/FR/RL/RR：只保持前三轮，右后6020由公共函数写零电压。 */
void ChassisApp_UpdateSteerHold(float dt) { (void)update_steer_zero_hold(dt, 3u); }

static float front_pair_limit(float value, float limit)
{
    if (value > limit) { return limit; }
    if (value < -limit) { return -limit; }
    return value;
}

static int front_pair_configuration_valid(void)
{
    const MotorPair_Cfg_t *cfg = &MOTORPAIR_CFG_CHASSIS_FRONT;
    uint8_t i;
    if (!finite_value(cfg->traj_v_max) || (cfg->traj_v_max <= 0.0f) ||
        !finite_value(cfg->traj_a_max) || (cfg->traj_a_max <= 0.0f) ||
        !finite_value(cfg->sync_kp) || (cfg->sync_kp < 0.0f) ||
        !finite_value(cfg->sync_speed_max) || (cfg->sync_speed_max < 0.0f) ||
        !finite_value(cfg->command_a_max) || (cfg->command_a_max <= 0.0f) ||
        !finite_value(CHASSIS_FRONT_PAIR_POSITION_TOL_ROTOR_TURNS) ||
        (CHASSIS_FRONT_PAIR_POSITION_TOL_ROTOR_TURNS < 0.0f)) { return 0; }
    for (i = 0u; i < 2u; i++) {
        const ChassisFront_Axis_t *axis = &g_chassis_front_axes[i];
        const steering_gear_t *drive = &g_chassis_wheels[i].module->steering_gear;
        if ((axis->motor->model != &DJI_MODEL_3508) ||
            !finite_value(axis->cascade.pos.out_max) || (axis->cascade.pos.out_max <= 0.0f) ||
            ((drive->encoder_direction != 1) && (drive->encoder_direction != -1)) ||
            ((drive->electric_direction != 1) && (drive->electric_direction != -1))) { return 0; }
    }
    return 1;
}

ChassisFront_StartResult_t ChassisApp_MoveFrontPairTurns(float left_rotor_turns,
                                                       float right_rotor_turns)
{
    return ChassisApp_MoveFrontPairTurnsWithPause(left_rotor_turns, right_rotor_turns, 0.0f);
}

ChassisFront_StartResult_t ChassisApp_MoveFrontPairTurnsWithPause(float left_rotor_turns,
                                                               float right_rotor_turns,
                                                               float pause_fraction)
{
    float turns[2];
    float distance;
    float from[2], target[2];
    uint8_t i;
    if (g_chassis_front_state == CHASSIS_FRONT_MOVING) { return CHASSIS_FRONT_START_BUSY; }
    if (!finite_value(pause_fraction) || (pause_fraction < 0.0f) || (pause_fraction >= 1.0f) ||
        (g_chassis_front_state == CHASSIS_FRONT_FAULT) ||
        !finite_value(left_rotor_turns) || !finite_value(right_rotor_turns) ||
        (left_rotor_turns == 0.0f) || (right_rotor_turns == 0.0f) ||
        (fabsf(fabsf(left_rotor_turns) - fabsf(right_rotor_turns)) > 0.0001f) ||
        !front_pair_configuration_valid()) { return CHASSIS_FRONT_START_BAD_CONFIG; }
    distance = fabsf(left_rotor_turns) * (CHASSIS_TWO_PI / DJI_MODEL_3508.gear_ratio);
    if (!finite_value(distance)) { return CHASSIS_FRONT_START_BAD_CONFIG; }
    turns[0] = left_rotor_turns;
    turns[1] = right_rotor_turns;
    /* Check all four before committing either wheel's relative origin. */
    for (i = 0u; i < 2u; i++) {
        const ChassisApp_Wheel_t *wheel = &g_chassis_wheels[i];
        if (!g_chassis_front_axes[i].seeded || !wheel->feedback_ready ||
            !DjiMotor_IsOnline(wheel->drive_motor) || !DjiMotor_IsOnline(wheel->steer_motor)) {
            return CHASSIS_FRONT_START_NOT_READY;
        }
        if (!finite_value(wheel->drive_motor->angle_rad) ||
            !finite_value(wheel->drive_motor->speed_rad_s)) {
            return CHASSIS_FRONT_START_BAD_CONFIG;
        }
        from[i] = (float)wheel->module->steering_gear.encoder_direction * wheel->drive_motor->angle_rad;
        target[i] = from[i] + (float)wheel->module->steering_gear.encoder_direction *
                    turns[i] * (CHASSIS_TWO_PI / DJI_MODEL_3508.gear_ratio);
        if (!finite_value(target[i])) { return CHASSIS_FRONT_START_BAD_CONFIG; }
    }
    memset(&g_chassis_front_pair, 0, sizeof(g_chassis_front_pair));
    g_chassis_front_pair.start_ms = HAL_GetTick();
    g_chassis_front_pair.pause_fraction = pause_fraction;
    g_chassis_front_pair.pause_ms = UINT32_MAX;
    g_chassis_front_pair.resume_ms = UINT32_MAX;
    s_front_pair_full_distance = distance;
    s_front_pair_pause_armed = (uint8_t)(pause_fraction > 0.0f);
    Traj_Init(&s_front_pair_traj, MOTORPAIR_CFG_CHASSIS_FRONT.traj_v_max,
              MOTORPAIR_CFG_CHASSIS_FRONT.traj_a_max);
    Traj_Start(&s_front_pair_traj, 0.0f, s_front_pair_pause_armed ? distance * pause_fraction : distance);
    for (i = 0u; i < 2u; i++) {
        ChassisFront_Axis_t *axis = &g_chassis_front_axes[i];
        float encoder = (float)g_chassis_wheels[i].module->steering_gear.encoder_direction;
        float position = from[i];
        float speed = encoder * axis->motor->speed_rad_s;
        s_front_pair_from[i] = position;
        s_front_pair_turns[i] = turns[i];
        s_front_pair_direction[i] = encoder * (turns[i] < 0.0f ? -1.0f : 1.0f);
        s_front_pair_speed[i] = front_pair_limit(speed, axis->cascade.pos.out_max);
        Cascade_Reset(&axis->cascade, position, speed);
        Traj_Start(&axis->traj, position, target[i]);
        g_chassis_front_pair.error_turns[i] = turns[i];
    }
    g_chassis_front_pair.planner_active = 1u;
    s_front_pair_sequence++;
    g_chassis_front_pair.move_sequence = s_front_pair_sequence;
    g_chassis_front_state = CHASSIS_FRONT_MOVING;
    return CHASSIS_FRONT_START_OK;
}

int ChassisApp_ResumeFrontPair(void)
{
    if ((g_chassis_front_state != CHASSIS_FRONT_MOVING) ||
        (s_front_pair_pause_armed == 0u) || (g_chassis_front_pair.paused == 0u)) { return 0; }
    /* 延续原始起点和完整终点，不重发一段相对行程。 */
    Traj_Start(&s_front_pair_traj, s_front_pair_traj.pos_ref, s_front_pair_full_distance);
    s_front_pair_pause_armed = 0u;
    g_chassis_front_pair.paused = 0u;
    g_chassis_front_pair.resume_ms = HAL_GetTick() - g_chassis_front_pair.start_ms;
    g_chassis_front_pair.planner_active = 1u;
    return 1;
}

static void front_pair_fault(ChassisApp_Status_t status, uint8_t motor_id)
{
    uint8_t i;
    if (g_chassis_front_state != CHASSIS_FRONT_FAULT) {
        s_front_pair_fault_status = status;
        g_chassis_front_pair.fault_motor_id = motor_id;
        if (g_chassis_front_state == CHASSIS_FRONT_MOVING) {
            g_chassis_front_pair.elapsed_ms = HAL_GetTick() - g_chassis_front_pair.start_ms;
        }
    }
    g_chassis_front_state = CHASSIS_FRONT_FAULT;
    g_chassis_app_status = status;
    Traj_Stop(&s_front_pair_traj);
    g_chassis_front_pair.planner_active = 0u;
    g_chassis_front_pair.paused = 0u;
    g_chassis_front_pair.correction_rad_s = 0.0f;
    for (i = 0u; i < 2u; i++) {
        ChassisFront_Axis_t *axis = &g_chassis_front_axes[i];
        Traj_Stop(&axis->traj);
        Cascade_Reset(&axis->cascade, axis->traj.pos_ref, 0.0f);
        s_front_pair_speed[i] = 0.0f;
        g_chassis_front_pair.speed_command_rpm[i] = 0.0f;
        axis->motor->cmd = 0;
    }
}

void ChassisApp_UpdateFrontPair(float dt)
{
    float position[2] = {0.0f, 0.0f}, speed[2] = {0.0f, 0.0f};
    float q[2] = {0.0f, 0.0f};
    float correction = 0.0f;
    const MotorPair_Cfg_t *cfg = &MOTORPAIR_CFG_CHASSIS_FRONT;
    uint8_t i;
    uint8_t drive_ready = 1u;

    for (i = 2u; i < SWERVE_MODULE_COUNT; i++) {
        g_chassis_wheels[i].drive_motor->cmd = 0;
    }
    /* Hold FL/FR/RL even on a drive fault; RR stays released with zero voltage.
     * Rear steering faults do not add front-drive readiness gates. */
    g_chassis_app_status = update_steer_zero_hold(dt, 3u);
    if (!finite_value(dt) || (dt <= 0.0f) || !front_pair_configuration_valid()) {
        front_pair_fault(!finite_value(dt) || dt <= 0.0f ?
                         CHASSIS_APP_FAULT_DT : CHASSIS_APP_FAULT_CONFIG, 0u);
    }
    for (i = 0u; i < 2u; i++) {
        ChassisFront_Axis_t *axis = &g_chassis_front_axes[i];
        float encoder = (float)g_chassis_wheels[i].module->steering_gear.encoder_direction;
        if (!DjiMotor_IsOnline(axis->motor)) {
            drive_ready = 0u;
            axis->motor->cmd = 0;
            axis->seeded = 0u;
            if ((g_chassis_front_state == CHASSIS_FRONT_MOVING) ||
                (g_chassis_front_state == CHASSIS_FRONT_DONE)) {
                front_pair_fault(CHASSIS_APP_FAULT_FEEDBACK, axis->motor->id);
            }
            continue;
        }
        DjiMotor_Update(axis->motor);
        position[i] = encoder * axis->motor->angle_rad;
        speed[i] = encoder * axis->motor->speed_rad_s;
        if (axis->seeded == 0u) {
            Traj_Start(&axis->traj, position[i], position[i]);
            Traj_Stop(&axis->traj);
            Cascade_Reset(&axis->cascade, position[i], speed[i]);
            axis->seeded = 1u;
        }
    }
    if (((g_chassis_front_state == CHASSIS_FRONT_MOVING) ||
         (g_chassis_front_state == CHASSIS_FRONT_DONE)) &&
        (g_chassis_app_status != CHASSIS_APP_READY)) {
        uint8_t id = !g_chassis_wheels[0].feedback_ready ?
                     g_chassis_wheels[0].steer_motor->id : g_chassis_wheels[1].steer_motor->id;
        front_pair_fault(g_chassis_app_status, id);
    }
    if (!drive_ready && (g_chassis_front_state == CHASSIS_FRONT_HOLD)) {
        g_chassis_app_status = CHASSIS_APP_FAULT_FEEDBACK;
    }
    if (g_chassis_front_state == CHASSIS_FRONT_MOVING) {
        g_chassis_front_pair.elapsed_ms = HAL_GetTick() - g_chassis_front_pair.start_ms;
        for (i = 0u; i < 2u; i++) {
            float encoder = (float)g_chassis_wheels[i].module->steering_gear.encoder_direction;
            q[i] = s_front_pair_direction[i] * (position[i] - s_front_pair_from[i]);
            g_chassis_front_pair.actual_turns[i] = encoder *
                (position[i] - s_front_pair_from[i]) * DJI_MODEL_3508.gear_ratio / CHASSIS_TWO_PI;
            g_chassis_front_pair.error_turns[i] = s_front_pair_turns[i] -
                                                g_chassis_front_pair.actual_turns[i];
        }
        g_chassis_front_pair.sync_error_turns = (q[0] - q[1]) *
                                              DJI_MODEL_3508.gear_ratio / CHASSIS_TWO_PI;
        if (s_front_pair_pause_armed != 0u) {
            float stop = s_front_pair_full_distance * g_chassis_front_pair.pause_fraction;
            float tol = CHASSIS_FRONT_PAIR_POSITION_TOL_ROTOR_TURNS * CHASSIS_TWO_PI / DJI_MODEL_3508.gear_ratio;
            g_chassis_front_pair.paused = (uint8_t)((s_front_pair_traj.active == 0u) &&
                (fabsf(q[0] - stop) <= tol) && (fabsf(q[1] - stop) <= tol) &&
                (fabsf(speed[0]) <= CHASSIS_FRONT_MOVE_SPEED_TOL_RAD_S) &&
                (fabsf(speed[1]) <= CHASSIS_FRONT_MOVE_SPEED_TOL_RAD_S));
            if (g_chassis_front_pair.paused && g_chassis_front_pair.pause_ms == UINT32_MAX) {
                g_chassis_front_pair.pause_ms = g_chassis_front_pair.elapsed_ms;
            }
        }
        if ((s_front_pair_pause_armed == 0u) &&
            ((fabsf(g_chassis_front_pair.error_turns[0]) <= CHASSIS_FRONT_PAIR_POSITION_TOL_ROTOR_TURNS) ||
             (fabsf(g_chassis_front_pair.error_turns[1]) <= CHASSIS_FRONT_PAIR_POSITION_TOL_ROTOR_TURNS))) {
            uint8_t winner = (uint8_t)(fabsf(g_chassis_front_pair.error_turns[0]) <=
                             CHASSIS_FRONT_PAIR_POSITION_TOL_ROTOR_TURNS ? 0u : 1u);
            g_chassis_front_pair.finish_motor_id = g_chassis_front_axes[winner].motor->id;
            g_chassis_front_state = CHASSIS_FRONT_DONE;
            s_front_pair_traj.pos_ref = s_front_pair_traj.target;
            Traj_Stop(&s_front_pair_traj);
            for (i = 0u; i < 2u; i++) {
                ChassisFront_Axis_t *axis = &g_chassis_front_axes[i];
                Traj_Start(&axis->traj, position[i], position[i]);
                Traj_Stop(&axis->traj);
                Cascade_Reset(&axis->cascade, position[i], speed[i]);
                s_front_pair_speed[i] = 0.0f;
            }
        } else {
            Traj_Update(&s_front_pair_traj, dt);
            correction = front_pair_limit(cfg->sync_kp * (q[0] - q[1]), cfg->sync_speed_max);
        }
    }
    g_chassis_front_pair.reference_turns = s_front_pair_traj.pos_ref *
                                         DJI_MODEL_3508.gear_ratio / CHASSIS_TWO_PI;
    g_chassis_front_pair.planner_active = s_front_pair_traj.active;
    g_chassis_front_pair.correction_rad_s = correction;
    for (i = 0u; i < 2u; i++) {
        ChassisFront_Axis_t *axis = &g_chassis_front_axes[i];
        const steering_gear_t *drive = &g_chassis_wheels[i].module->steering_gear;
        float desired, output;
        /* Before an accepted drive request, read encoders but run no drive PID.
         * Otherwise the upper mechanism rolling these wheels creates a brake. */
        if ((g_chassis_front_state == CHASSIS_FRONT_HOLD) ||
            (g_chassis_front_state == CHASSIS_FRONT_FAULT) || !DjiMotor_IsOnline(axis->motor)) {
            axis->motor->cmd = 0;
            s_front_pair_speed[i] = 0.0f;
            g_chassis_front_pair.speed_command_rpm[i] = 0.0f;
            continue;
        }
        if (g_chassis_front_state == CHASSIS_FRONT_MOVING) {
            float common_speed;
            axis->traj.pos_ref = s_front_pair_from[i] + s_front_pair_direction[i] * s_front_pair_traj.pos_ref;
            axis->traj.vel_ref = s_front_pair_direction[i] * s_front_pair_traj.vel_ref;
            axis->traj.active = s_front_pair_traj.active;
            axis->cascade.pos.setpoint = axis->traj.pos_ref;
            common_speed = s_front_pair_direction[i] *
                PID_Update_Continuous(&axis->cascade.pos, position[i], dt) + s_front_pair_traj.vel_ref;
            common_speed += i == 0u ? -correction : correction;
            if (s_front_pair_traj.active && common_speed < 0.0f) { common_speed = 0.0f; }
            desired = s_front_pair_direction[i] * front_pair_limit(common_speed, axis->cascade.pos.out_max);
            s_front_pair_speed[i] += front_pair_limit(desired - s_front_pair_speed[i], cfg->command_a_max * dt);
            axis->cascade.spd.setpoint = s_front_pair_speed[i];
            output = PID_Update_Continuous(&axis->cascade.spd, speed[i], dt);
        } else {
            /* Only latched DONE holds its captured endpoint, no cross coupling. */
            output = Cascade_Update(&axis->cascade, position[i], speed[i], axis->traj.target, 0.0f, dt);
            s_front_pair_speed[i] = axis->cascade.spd.setpoint;
        }
        if (!finite_value(output)) {
            front_pair_fault(CHASSIS_APP_FAULT_OUTPUT, axis->motor->id);
            break;
        }
        axis->motor->cmd = raw_command(output, drive->electric_direction, axis->motor->model->cmd_max);
        g_chassis_front_pair.speed_command_rpm[i] = s_front_pair_speed[i] *
            (float)drive->encoder_direction * DJI_MODEL_3508.gear_ratio * 60.0f / CHASSIS_TWO_PI;
    }
    if (g_chassis_front_state == CHASSIS_FRONT_FAULT) {
        g_chassis_app_status = s_front_pair_fault_status;
    }
    (void)DjiMotor_SendGroup(&hfdcan1, 0u);
    (void)DjiMotor_SendGroup(&hfdcan2, 0u);
}

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
void ChassisApp_UpdateFrontSteerHold(float dt) { (void)dt; }
void ChassisApp_UpdateSteerHold(float dt) { (void)dt; }
void ChassisApp_UpdateFrontPair(float dt) { (void)dt; }
ChassisFront_StartResult_t ChassisApp_MoveFrontPairTurnsWithPause(float l, float r, float p)
{ (void)l; (void)r; (void)p; return CHASSIS_FRONT_START_BAD_CONFIG; }
int ChassisApp_ResumeFrontPair(void) { return 0; }
ChassisFront_StartResult_t ChassisApp_MoveFrontPairTurns(float left, float right)
{
    (void)left; (void)right;
    return CHASSIS_FRONT_START_BAD_CONFIG;
}
ChassisFront_StartResult_t ChassisApp_MoveFrontTurns(float left_rotor_turns,
                                                   float right_rotor_turns)
{
    (void)left_rotor_turns; (void)right_rotor_turns;
    return CHASSIS_FRONT_START_BAD_CONFIG;
}

#endif /* CHASSIS_ENABLE_SWERVE */
