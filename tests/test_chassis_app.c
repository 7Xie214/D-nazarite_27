#include "ChassisApp.h"
#include "Chassis.h"
#include "DjiMotor.h"
#include "MotorApp.h"
#include "My_CAN.h"
#include "fdcan.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
uint32_t test_irq_mask;
static uint32_t tick = 100u;
static unsigned int checks, failures, frame_count;
static DjiMotor_t *inject_motor;
static uint16_t inject_raw;
static int32_t inject_consumed_raw;
static uint8_t inject_pending;
static struct {
    FDCAN_HandleTypeDef *bus;
    uint16_t id;
    uint8_t data[8];
} frames[16];

uint32_t HAL_GetTick(void)
{
    /* Simulate one CAN IRQ after Update consumed a raw value, before APP seeds. */
    if ((inject_pending != 0u) && (inject_motor->angle_seeded != 0u) &&
        (inject_motor->prev_raw_angle == inject_consumed_raw)) {
        uint8_t data[8] = {0u};
        inject_pending = 0u;
        data[0] = (uint8_t)(inject_raw >> 8u);
        data[1] = (uint8_t)(inject_raw & 0xFFu);
        DjiMotor_OnCanFrame(inject_motor->hfdcan,
                           (uint16_t)(inject_motor->model->feedback_base + inject_motor->id), data, 8u);
    }
    return tick;
}
uint32_t osKernelGetTickCount(void) { return tick; }
void Test_EncoderOnCanFrame(FDCAN_HandleTypeDef *bus, uint16_t id,
                           const uint8_t *data, uint8_t len)
{
    (void)bus; (void)id; (void)data; (void)len;
}
HAL_StatusTypeDef BSP_CAN_Send(FDCAN_HandleTypeDef *bus, uint16_t id,
                               const uint8_t *data, uint8_t len)
{
    if ((frame_count >= 16u) || (len != 8u)) { failures++; return HAL_ERROR; }
    frames[frame_count].bus = bus;
    frames[frame_count].id = id;
    memcpy(frames[frame_count].data, data, 8u);
    frame_count++;
    return HAL_OK;
}

#define CHECK(condition) do { checks++; if (!(condition)) { \
    printf("FAIL line %d: %s\n", __LINE__, #condition); failures++; } } while (0)
#define NEAR(actual, expected) CHECK(fabsf((actual) - (expected)) < 0.0001f)

#if CHASSIS_ENABLE_SWERVE
static void feed(DjiMotor_t *motor, uint16_t raw, int16_t rpm)
{
    uint16_t speed = (uint16_t)rpm;
    uint8_t data[8] = {0u};
    data[0] = (uint8_t)(raw >> 8u);
    data[1] = (uint8_t)(raw & 0xFFu);
    data[2] = (uint8_t)(speed >> 8u);
    data[3] = (uint8_t)(speed & 0xFFu);
    DjiMotor_OnCanFrame(motor->hfdcan,
                       (uint16_t)(motor->model->feedback_base + motor->id), data, 8u);
}

static void feed_all(uint16_t raw, int16_t drive_rpm)
{
    uint8_t i;
    for (i = 0u; i < 4u; i++) {
        feed(g_chassis_wheels[i].drive_motor, 0u, drive_rpm);
        feed(g_chassis_wheels[i].steer_motor, raw, 0);
    }
}

static void check_chassis_frames(void)
{
    uint8_t i;
    CHECK(frame_count == 4u);
    for (i = 0u; i < frame_count; i++) {
        CHECK((frames[i].bus == &hfdcan1) || (frames[i].bus == &hfdcan2));
        CHECK((frames[i].id == 0x200u) || (frames[i].id == 0x1FFu));
    }
}

static void zero_commands(void)
{
    uint8_t i;
    for (i = 0u; i < 4u; i++) {
        CHECK(g_chassis_wheels[i].drive_motor->cmd == 0);
        CHECK(g_chassis_wheels[i].steer_motor->cmd == 0);
    }
}

static void check_packet(FDCAN_HandleTypeDef *bus, uint16_t id,
                          int16_t a, int16_t b, int16_t c, int16_t d)
{
    const int16_t values[4] = {a, b, c, d};
    unsigned int f, found = 0u;
    uint8_t i;
    for (f = 0u; f < frame_count; f++) {
        if ((frames[f].bus != bus) || (frames[f].id != id)) { continue; }
        found++;
        for (i = 0u; i < 4u; i++) {
            uint16_t raw = (uint16_t)values[i];
            CHECK(frames[f].data[i * 2u] == (uint8_t)(raw >> 8u));
            CHECK(frames[f].data[i * 2u + 1u] == (uint8_t)(raw & 0xFFu));
        }
    }
    CHECK(found == 1u);
}

static void tune(void)
{
    uint8_t i;
    for (i = 0u; i < 4u; i++) {
        g_chassis_wheels[i].drive_pid.Kp = 1000.0f;
        g_chassis_wheels[i].steer_pid.pos.Kp = 1.0f;
        g_chassis_wheels[i].steer_pid.pos.out_max = 3.0f;
        g_chassis_wheels[i].steer_pid.spd.Kp = 1000.0f;
    }
}

static void restart(void)
{
    Chassis_Init();
    ChassisApp_Init();
    tune();
    frame_count = 0u;
}

static void test_conversions_and_control(void)
{
    uint8_t i;
    const float pi = 3.14159265358979323846f;
    float old_angle;
    restart();
    feed_all(512u, 0);
    ChassisApp_Update(0.001f);
    for (i = 0u; i < 4u; i++) {
        NEAR(g_swerve_modules[i].actual_angle_rad, pi / 8.0f);
        NEAR(g_swerve_modules[i].target_angle_rad, pi / 8.0f);
    }
    zero_commands(); /* First feedback holds measured heading, not zero. */
    restart();
    feed_all(0u, 0);
    ChassisApp_Update(0.001f);
    frame_count = 0u;
    ChassisApp_SetVelocity(-1.0f, 0.0f, 0.0f);
    ChassisApp_Update(0.001f);
    for (i = 0u; i < 4u; i++) {
        NEAR(g_swerve_modules[i].target_angle_rad, 0.0f);
        NEAR(g_swerve_modules[i].target_speed_mps, -1.0f);
        CHECK(g_chassis_wheels[i].drive_motor->cmd == -1000);
    }
    frame_count = 0u;
    ChassisApp_SetVelocity(0.0f, 1.0f, 0.0f);
    ChassisApp_Update(0.001f);
    for (i = 0u; i < 4u; i++) {
        NEAR(g_swerve_modules[i].target_angle_rad, pi / 2.0f);
        CHECK(g_chassis_wheels[i].steer_motor->cmd > 1500);
        CHECK(g_chassis_wheels[i].steer_motor->cmd < 1600);
    }
    frame_count = 0u;
    ChassisApp_SetVelocity(0.0f, 0.0f, 0.0f);
    ChassisApp_Update(0.001f);
    for (i = 0u; i < 4u; i++) {
        NEAR(g_swerve_modules[i].target_angle_rad, pi / 2.0f);
        CHECK(g_chassis_wheels[i].drive_motor->cmd == 0);
    }

    restart();
    for (i = 0u; i < 4u; i++) {
        g_swerve_modules[i].steering_gear.encoder_direction = -1;
        g_swerve_modules[i].steering_gear.electric_direction = -1;
    }
    feed_all(0u, 500);
    ChassisApp_SetVelocity(1.0f, 0.0f, 0.0f);
    ChassisApp_Update(0.001f);
    for (i = 0u; i < 4u; i++) {
        float expected = -500.0f / DJI_MODEL_3508.gear_ratio * (2.0f * pi / 60.0f) * 0.1f;
        NEAR(g_chassis_wheels[i].drive_speed_mps, expected);
        CHECK(g_chassis_wheels[i].drive_motor->cmd == (int16_t)(-1000.0f * (1.0f - expected)));
    }
    restart();
    for (i = 0u; i < 4u; i++) {
        g_swerve_modules[i].rudder_direction.zero_offset = 1000u;
        g_swerve_modules[i].rudder_direction.encoder_direction = -1;
        g_swerve_modules[i].rudder_direction.electirc_direction = -1;
        feed(g_chassis_wheels[i].drive_motor, 0u, 0);
        feed(g_chassis_wheels[i].steer_motor, 1000u, 120);
    }
    ChassisApp_Update(0.001f);
    for (i = 0u; i < 4u; i++) {
        NEAR(g_swerve_modules[i].actual_angle_rad, 0.0f);
        NEAR(g_chassis_wheels[i].steer_speed_rad_s, -4.0f * pi);
        CHECK(g_chassis_wheels[i].steer_motor->cmd < -12000);
    }

    restart();
    for (i = 0u; i < 4u; i++) {
        g_swerve_modules[i].angle_range = SWERVE_ANGLE_0_TO_180;
    }
    feed_all(4096u, 0);
    ChassisApp_Update(0.001f);
    for (i = 0u; i < 4u; i++) {
        NEAR(g_swerve_modules[i].actual_angle_rad, pi);
        NEAR(g_swerve_modules[i].target_angle_rad, pi);
    }
    zero_commands();
    frame_count = 0u;
    feed_all(6144u, 0);
    ChassisApp_SetVelocity(1.0f, 0.0f, 0.0f);
    ChassisApp_Update(0.001f);
    zero_commands(); /* 270 degrees cannot be clamped to a legal angle. */

    restart();
    for (i = 0u; i < 4u; i++) {
        g_swerve_modules[i].angle_range = SWERVE_ANGLE_CONTINUOUS;
    }
    feed_all(8190u, 0);
    ChassisApp_Update(0.001f);
    old_angle = g_swerve_modules[0].actual_angle_rad;
    frame_count = 0u;
    feed_all(2u, 0);
    ChassisApp_Update(0.001f);
    NEAR(g_swerve_modules[0].actual_angle_rad - old_angle, 4.0f * 2.0f * pi / 8192.0f);

    restart();
    feed_all(0u, 0);
    ChassisApp_Update(0.001f);
    frame_count = 0u;
    g_chassis_wheels[0].drive_pid.Kp = 0.0f;
    g_chassis_wheels[0].drive_pid.Ki = 1000.0f;
    g_chassis_wheels[0].drive_pid.integral_max = 10.0f;
    ChassisApp_SetVelocity(1.0f, 0.0f, 0.0f);
    ChassisApp_Update(0.1f);
    CHECK(g_chassis_wheels[0].drive_motor->cmd == 100);
    CHECK(g_chassis_wheels[1].drive_motor->cmd == 1000);
    NEAR(g_chassis_wheels[1].drive_pid.integral, 0.0f);
    frame_count = 0u;
    ChassisApp_Update(0.0f);
    zero_commands();
    NEAR(g_chassis_wheels[0].drive_pid.integral, 0.0f);
    frame_count = 0u;
    ChassisApp_SetVelocity(NAN, 0.0f, 0.0f);
    ChassisApp_Update(0.001f);
    zero_commands();
    test_irq_mask = 1u;
    ChassisApp_SetVelocity(0.0f, 0.0f, 0.0f);
    CHECK(test_irq_mask == 1u);
    frame_count = 0u;
    ChassisApp_Update(0.001f);
    CHECK(test_irq_mask == 1u);
    test_irq_mask = 0u;

    restart();
    feed_all(0u, 0);
    ChassisApp_SetVelocity(1.0f, 0.0f, 0.0f);
    ChassisApp_Update(0.001f);
    frame_count = 0u;
    tick += 51u;
    for (i = 0u; i < 4u; i++) {
        feed(g_chassis_wheels[i].drive_motor, 0u, 0);
        if (i != 2u) { feed(g_chassis_wheels[i].steer_motor, 0u, 0); }
    }
    ChassisApp_Update(0.001f);
    zero_commands(); /* One missing steering motor stops all eight. */
    CHECK(g_chassis_app_status == CHASSIS_APP_FAULT_FEEDBACK);
    frame_count = 0u;
    feed_all(1024u, 0);
    ChassisApp_SetVelocity(0.0f, 0.0f, 0.0f);
    ChassisApp_Update(0.001f);
    zero_commands();
    CHECK(g_chassis_app_status == CHASSIS_APP_READY);
    for (i = 0u; i < 4u; i++) {
        NEAR(g_swerve_modules[i].target_angle_rad, pi / 4.0f);
    }
    restart();
    feed_all(0u, 0);
    ChassisApp_Update(0.001f);
    frame_count = 0u;
    for (i = 0u; i < 4u; i++) {
        g_chassis_wheels[i].drive_pid.Kp = 1.0e6f;
        g_chassis_wheels[i].drive_pid.out_max = 1.0e6f;
        g_chassis_wheels[i].steer_pid.spd.Kp = 1.0e6f;
        g_chassis_wheels[i].steer_pid.spd.out_max = 1.0e6f;
    }
    ChassisApp_SetVelocity(0.0f, 1.0f, 0.0f);
    ChassisApp_Update(0.001f);
    for (i = 0u; i < 4u; i++) {
        CHECK(g_chassis_wheels[i].drive_motor->cmd == 16384);
        CHECK(g_chassis_wheels[i].steer_motor->cmd == 30000);
    }
    frame_count = 0u;
    g_chassis_wheels[3].drive_pid.Kp = NAN;
    ChassisApp_Update(0.001f);
    zero_commands();
    CHECK(g_chassis_app_status == CHASSIS_APP_FAULT_OUTPUT);

    restart();
    for (i = 0u; i < 4u; i++) { g_swerve_modules[i].angle_range = SWERVE_ANGLE_CONTINUOUS; }
    feed_all(1000u, 0);
    inject_motor = g_chassis_wheels[0].steer_motor;
    inject_motor->angle_seeded = 0u;
    inject_raw = 1500u;
    inject_consumed_raw = 1000;
    inject_pending = 1u;
    ChassisApp_Update(0.001f);
    CHECK(inject_pending == 0u);
    NEAR(g_swerve_modules[0].actual_angle_rad, 1000.0f * 2.0f * pi / 8192.0f);
    frame_count = 0u;
    ChassisApp_Update(0.001f);
    NEAR(g_swerve_modules[0].actual_angle_rad, 1500.0f * 2.0f * pi / 8192.0f);

    restart();
    for (i = 0u; i < 4u; i++) { g_swerve_modules[i].angle_range = SWERVE_ANGLE_CONTINUOUS; }
    feed_all(0u, 0);
    inject_motor = g_chassis_wheels[0].steer_motor;
    inject_motor->rx_count = 0u;
    inject_motor->raw_angle = 0u;
    inject_motor->angle_seeded = 0u;
    inject_raw = 1500u;
    inject_consumed_raw = 0;
    inject_pending = 1u;
    ChassisApp_Update(0.001f);
    CHECK(inject_pending == 0u);
    zero_commands();
    CHECK(g_chassis_wheels[0].feedback_ready == 0u);
    frame_count = 0u;
    ChassisApp_Update(0.001f);
    NEAR(g_swerve_modules[0].target_angle_rad, 1500.0f * 2.0f * pi / 8192.0f);
    zero_commands();
}

static void test_front_motion(void)
{
    uint8_t i;
    float left_start, right_start, left_target, right_target;
    const float two_pi = 6.28318530718f;
    Chassis_Init();
    ChassisApp_Init();
    feed_all(0u, 0);
    frame_count = 0u;
    ChassisApp_UpdateFrontMove(0.001f);
    CHECK(g_chassis_app_status == CHASSIS_APP_READY);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_HOLD);
    CHECK(g_chassis_front_axes[0].motor == &g_motor_3508_4);
    CHECK(g_chassis_front_axes[1].motor == &g_motor_3508_5);
    CHECK(ChassisApp_MoveFrontTurns(0.0f, 0.0f) == CHASSIS_FRONT_START_BAD_CONFIG);
    CHECK(ChassisApp_MoveFrontTurns(1.0f, 2.0f) == CHASSIS_FRONT_START_BAD_CONFIG);
    for (i = 0u; i < 2u; i++) {
        g_chassis_front_axes[i].cascade.pos.Kp = 10.0f;
        g_chassis_front_axes[i].cascade.pos.out_max = 2.0f;
        g_chassis_front_axes[i].cascade.spd.Kp = 1000.0f;
        g_chassis_front_axes[i].traj.v_max = 1.0f;
        g_chassis_front_axes[i].traj.a_max = 2.0f;
    }
    left_start = g_chassis_front_axes[0].traj.pos_ref;
    right_start = g_chassis_front_axes[1].traj.pos_ref;
    CHECK(ChassisApp_MoveFrontTurns(1.0f, 2.0f) == CHASSIS_FRONT_START_OK);
    left_target = g_chassis_front_axes[0].traj.target;
    right_target = g_chassis_front_axes[1].traj.target;
    NEAR(left_target - left_start, two_pi / DJI_MODEL_3508.gear_ratio);
    NEAR(right_target - right_start, 2.0f * two_pi / DJI_MODEL_3508.gear_ratio);
    CHECK(ChassisApp_MoveFrontTurns(9.0f, 9.0f) == CHASSIS_FRONT_START_BUSY);
    NEAR(g_chassis_front_axes[0].traj.target, left_target);
    frame_count = 0u;
    ChassisApp_UpdateFrontMove(0.001f);
    CHECK(g_chassis_front_axes[0].traj.pos_ref > left_start);
    CHECK(g_chassis_front_axes[1].traj.pos_ref > right_start);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_MOVING);
    CHECK(g_motor_3508_6.cmd == 0 && g_motor_3508_7.cmd == 0);
    check_chassis_frames();

    /* Planner completion alone must not acknowledge physical arrival. */
    for (i = 0u; i < 2u; i++) {
        g_chassis_front_axes[i].traj.active = 0u;
        g_chassis_front_axes[i].traj.pos_ref = g_chassis_front_axes[i].traj.target;
        g_chassis_front_axes[i].traj.vel_ref = 0.0f;
    }
    frame_count = 0u;
    ChassisApp_UpdateFrontMove(0.001f);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_MOVING);
    g_motor_3508_4.total_counts += 8192;
    frame_count = 0u;
    ChassisApp_UpdateFrontMove(0.001f);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_MOVING);
    g_motor_3508_5.total_counts += 16384;
    feed(&g_motor_3508_4, 0u, 100);
    frame_count = 0u;
    ChassisApp_UpdateFrontMove(0.001f);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_MOVING);
    feed_all(0u, 0);
    frame_count = 0u;
    ChassisApp_UpdateFrontMove(0.1f);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_MOVING);
    /* A first in-tolerance sample cannot account for the previous 100 ms.
     * Leaving tolerance also restarts the continuous settling interval. */
    feed(&g_motor_3508_4, 0u, 100);
    frame_count = 0u;
    ChassisApp_UpdateFrontMove(0.001f);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_MOVING);
    feed_all(0u, 0);
    for (i = 0u; i < 25u; i++) {
        frame_count = 0u;
        ChassisApp_UpdateFrontMove(0.001f);
    }
    CHECK(g_chassis_front_state == CHASSIS_FRONT_DONE);
    CHECK(ChassisApp_MoveFrontTurns(-0.5f, 0.5f) == CHASSIS_FRONT_START_OK);
    NEAR(g_chassis_front_axes[0].traj.target, left_target - 0.5f * two_pi / DJI_MODEL_3508.gear_ratio);
    NEAR(g_chassis_front_axes[1].traj.target, right_target + 0.5f * two_pi / DJI_MODEL_3508.gear_ratio);
    g_motor_3508_4.rx_count = 0u;
    frame_count = 0u;
    ChassisApp_UpdateFrontMove(0.001f);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_FAULT);
    zero_commands();
    feed_all(0u, 0);
    frame_count = 0u;
    ChassisApp_UpdateFrontMove(0.001f);
    CHECK(g_chassis_front_axes[0].traj.active == 0u);
    NEAR(g_chassis_front_axes[0].traj.pos_ref, g_motor_3508_4.angle_rad);

    Chassis_Init();
    g_swerve_modules[0].steering_gear.encoder_direction = -1;
    g_swerve_modules[0].steering_gear.electric_direction = -1;
    ChassisApp_Init();
    feed_all(0u, 0);
    frame_count = 0u;
    ChassisApp_UpdateFrontMove(0.001f);
    for (i = 0u; i < 2u; i++) {
        g_chassis_front_axes[i].cascade.pos.Kp = 10.0f;
        g_chassis_front_axes[i].cascade.pos.out_max = 2.0f;
        g_chassis_front_axes[i].cascade.spd.Kp = 1000.0f;
        g_chassis_front_axes[i].traj.v_max = 1.0f;
        g_chassis_front_axes[i].traj.a_max = 2.0f;
    }
    left_start = -g_motor_3508_4.angle_rad;
    CHECK(ChassisApp_MoveFrontTurns(1.0f, 1.0f) == CHASSIS_FRONT_START_OK);
    NEAR(g_chassis_front_axes[0].traj.target - left_start, two_pi / DJI_MODEL_3508.gear_ratio);
    frame_count = 0u;
    ChassisApp_UpdateFrontMove(0.01f);
    CHECK(g_motor_3508_4.cmd < 0);
    CHECK(g_motor_3508_5.cmd > 0);

    /* An ideal encoder plant follows the real planner, including reverse travel
     * and wrap crossings; no forced planner completion in this scenario. */
    {
        uint16_t cycle;
        float initial[2];
        Chassis_Init();
        ChassisApp_Init();
        feed_all(0u, 0);
        frame_count = 0u;
        ChassisApp_UpdateFrontMove(0.001f);
        for (i = 0u; i < 2u; i++) {
            initial[i] = g_chassis_front_axes[i].traj.pos_ref;
            g_chassis_front_axes[i].cascade.pos.Kp = 10.0f;
            g_chassis_front_axes[i].cascade.pos.out_max = 2.0f;
            g_chassis_front_axes[i].cascade.spd.Kp = 1000.0f;
            g_chassis_front_axes[i].traj.v_max = 1.0f;
            g_chassis_front_axes[i].traj.a_max = 2.0f;
        }
        CHECK(ChassisApp_MoveFrontTurns(2.0f, -1.5f) == CHASSIS_FRONT_START_OK);
        for (cycle = 0u; cycle < 10000u && g_chassis_front_state == CHASSIS_FRONT_MOVING; cycle++) {
            tick++;
            feed_all(0u, 0);
            for (i = 0u; i < 2u; i++) {
                ChassisFront_Axis_t *axis = &g_chassis_front_axes[i];
                int32_t raw = (int32_t)lroundf((axis->traj.pos_ref - initial[i]) *
                              axis->motor->model->gear_ratio / two_pi * 8192.0f) % 8192;
                int16_t rpm = (int16_t)lroundf(axis->traj.vel_ref * axis->motor->model->gear_ratio / two_pi * 60.0f);
                if (raw < 0) { raw += 8192; }
                feed(axis->motor, (uint16_t)raw, rpm);
            }
            frame_count = 0u;
            ChassisApp_UpdateFrontMove(0.001f);
        }
        CHECK(g_chassis_front_state == CHASSIS_FRONT_DONE);
        CHECK(cycle > 20u && cycle < 10000u);
        NEAR(g_motor_3508_4.angle_rad - initial[0], 2.0f * two_pi / DJI_MODEL_3508.gear_ratio);
        NEAR(g_motor_3508_5.angle_rad - initial[1], -1.5f * two_pi / DJI_MODEL_3508.gear_ratio);
    }
}

int main(void)
{
    uint8_t i;
    DjiMotor_t *const drive[4] = {
        &g_motor_3508_4, &g_motor_3508_5, &g_motor_3508_6, &g_motor_3508_7
    };
    DjiMotor_t *const steer[4] = {
        &g_motor_6020_1, &g_motor_6020_4, &g_motor_6020_2, &g_motor_6020_3
    };
    DjiMotor_Init();
    Chassis_Init();
    ChassisApp_Init();
    MotorApp_Init();
    CHECK(g_djimotor_reg_fail == 0u);
    for (i = 0u; i < 4u; i++) {
        CHECK(g_chassis_wheels[i].module == &g_swerve_modules[i]);
        CHECK(g_chassis_wheels[i].drive_motor == drive[i]);
        CHECK(g_chassis_wheels[i].steer_motor == steer[i]);
    }
    /* No feedback is never sufficient to drive. */
    ChassisApp_SetVelocity(1.0f, 0.0f, 0.0f);
    ChassisApp_Update(0.001f);
    zero_commands();
    check_chassis_frames();
    frame_count = 0u;
    feed_all(0u, 0);
    ChassisApp_SetVelocity(0.0f, 0.0f, 0.0f);
    ChassisApp_Update(0.001f);
    zero_commands();
    check_chassis_frames();
    frame_count = 0u;
    if (CHASSIS_SWERVE_WHEEL_RADIUS_M > 0.0f) {
        /* Test tuning is local to runtime PID states, production gains stay zero. */
        tune();
        ChassisApp_SetVelocity(1.0f, 0.0f, 0.0f);
        ChassisApp_Update(0.001f);
        for (i = 0u; i < 4u; i++) {
            NEAR(g_swerve_modules[i].target_speed_mps, 1.0f);
            CHECK(drive[i]->cmd == 1000);
            CHECK(steer[i]->cmd == 0);
        }
        check_chassis_frames();
        check_packet(&hfdcan1, 0x200u, 1000, 1000, 0, 0);
        check_packet(&hfdcan2, 0x200u, 0, 0, 1000, 1000);
        check_packet(&hfdcan1, 0x1FFu, 0, 0, 0, 0);
        check_packet(&hfdcan2, 0x1FFu, 0, 0, 0, 0);
        frame_count = 0u;
        tick += 51u;
        ChassisApp_Update(0.001f);
        zero_commands();
        check_chassis_frames();
        test_conversions_and_control();
    } else {
        ChassisApp_SetVelocity(1.0f, 0.0f, 0.0f);
        ChassisApp_Update(0.001f);
        zero_commands();
        check_chassis_frames();
    }
    test_front_motion();
    printf("%u chassis application checks, %u failures\n", checks, failures);
    return (failures == 0u) ? 0 : 1;
}
#else
int main(void)
{
    DjiMotor_Init();
    Chassis_Init();
    ChassisApp_Init();
    ChassisApp_SetVelocity(1.0f, 0.0f, 0.0f);
    ChassisApp_Update(0.001f);
    CHECK(frame_count == 0u);
    CHECK(g_djimotor_reg_fail == 0u);
    printf("%u reserved chassis checks, %u failures\n", checks, failures);
    return (failures == 0u) ? 0 : 1;
}
#endif
