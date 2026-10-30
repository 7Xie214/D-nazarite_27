#include "MotorApp.h"
#include "PID_Config.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
static uint32_t tick = 100u;
static unsigned int failures;
static float position[5], speed[5]; /* rotor turns / rpm: synthetic encoder plant */
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); failures++; } } while (0)
uint32_t HAL_GetTick(void) { return tick; }
void Test_EncoderOnCanFrame(FDCAN_HandleTypeDef *can, uint16_t id,
                            const uint8_t *data, uint8_t len)
{ (void)can; (void)id; (void)data; (void)len; }
HAL_StatusTypeDef BSP_CAN_Send(FDCAN_HandleTypeDef *can, uint16_t id,
                              const uint8_t *data, uint8_t len)
{
    (void)can; CHECK(len == 8u);
    if (id == 0x200u) {
        int16_t c3 = (int16_t)((uint16_t)data[4] * 256u + data[5]);
        int16_t c4 = (int16_t)((uint16_t)data[6] * 256u + data[7]);
        CHECK(c3 == g_motor_2006_2.cmd && c4 == g_motor_2006_1.cmd);
    }
    return HAL_OK;
}
static void cycle(int missing)
{
    unsigned int i;
    tick++;
    for (i = 0u; i < 5u; i++) {
        Axis_t *a = MotorApp_GetAxis((uint8_t)i);
        int32_t counts = (int32_t)lroundf(position[i] * 8192.0f);
        uint16_t angle = (uint16_t)((uint32_t)counts & 8191u);
        uint16_t rpm = (uint16_t)(int16_t)speed[i];
        uint8_t data[8] = {(uint8_t)(angle >> 8), (uint8_t)angle,
                           (uint8_t)(rpm >> 8), (uint8_t)rpm, 0u, 0u, 30u, 0u};
        if ((int)i != missing) {
            DjiMotor_OnCanFrame(&hfdcan3, (uint16_t)(0x200u + a->motor->id), data, 8u);
        }
    }
    MotorApp_Update(0.001f);
}
static void place_pair(float q3, float q4)
{
    /* Only used to place feedback in test cases, not as a physical motor model. */
    position[4] = -q3; position[3] = q4;
    g_motor_2006_2.total_counts = (int32_t)lroundf(position[4] * 8192.0f);
    g_motor_2006_1.total_counts = (int32_t)lroundf(position[3] * 8192.0f);
    g_motor_2006_2.prev_raw_angle = (int32_t)((uint32_t)g_motor_2006_2.total_counts & 8191u);
    g_motor_2006_1.prev_raw_angle = (int32_t)((uint32_t)g_motor_2006_1.total_counts & 8191u);
}
int main(int argc, char **argv)
{
    unsigned int n;
    const MotorApp_2006PairStatus_t *s;
    Axis_t *a3, *a4;
    (void)argc;
    DjiMotor_Init(); MotorApp_Init(); cycle(-1);
    a3 = MotorApp_GetAxis(4u); a4 = MotorApp_GetAxis(3u);
    s = MotorApp_Get2006PairStatus();
    if (strcmp(argv[1], "atomic") == 0) {
        g_motor_2006_1.rx_count = 0u;
        CHECK(MotorApp_Move2006Pair(2.0f) == MOTORAPP_ERR_NOFB);
        CHECK(a3->state == AXIS_IDLE && a4->state == AXIS_IDLE);
        CHECK(s->state == MOTORAPP_PAIR_IDLE);
        cycle(-1);
        CHECK(MotorApp_Move2006Pair(2.0f) == MOTORAPP_OK);
        CHECK(a3->traj.target < 0.0f && a4->traj.target > 0.0f);
        CHECK(MotorApp_Move2006Pair(2.0f) == MOTORAPP_ERR_BUSY);
        CHECK(MotorApp_MoveTurns(a3, 1.0f) == MOTORAPP_ERR_BUSY);
        CHECK(MotorApp_MoveTurns(a4, 1.0f) == MOTORAPP_ERR_BUSY);
    } else if (strcmp(argv[1], "any_id3") == 0 || strcmp(argv[1], "any_id4") == 0 ||
               strcmp(argv[1], "any_reverse_id3") == 0 || strcmp(argv[1], "any_reverse_id4") == 0) {
        float sign = strstr(argv[1], "reverse") != NULL ? -1.0f : 1.0f;
        unsigned int winner = strstr(argv[1], "id3") != NULL ? 0u : 1u;
        float hold3, hold4;
        CHECK(MotorApp_Move2006Pair(sign * 5.0f) == MOTORAPP_OK);
        a3->cascade.spd.integral = 10.0f; a4->cascade.spd.integral = -10.0f;
        place_pair(sign * (winner == 0u ? 5.0f : 1.0f), sign * (winner == 1u ? 5.0f : 1.0f));
        cycle(-1);
        CHECK(s->state == MOTORAPP_PAIR_DONE);
        CHECK(a3->state == AXIS_IDLE && a4->state == AXIS_IDLE);
        CHECK(fabsf(a3->traj.target - a3->motor->angle_rad) < 1e-6f);
        CHECK(fabsf(a4->traj.target - a4->motor->angle_rad) < 1e-6f);
        CHECK(a3->cascade.spd.integral == 0.0f && a4->cascade.spd.integral == 0.0f);
        CHECK(a3->motor->cmd == 0 && a4->motor->cmd == 0);
        CHECK(s->correction_rad_s == 0.0f && a3->traj.active == 0u && a4->traj.active == 0u);
        CHECK(s->timing.first_in_tol_ms[winner] == 1u);
        CHECK(s->timing.finish_motor_id == (winner == 0u ? 3u : 4u));
        CHECK(s->timing.first_in_tol_ms[1u - winner] == UINT32_MAX);
        hold3 = a3->traj.target; hold4 = a4->traj.target;
        place_pair(sign * 6.0f, sign * 2.0f); cycle(-1);
        CHECK(s->state == MOTORAPP_PAIR_DONE); /* completion stays latched while holding */
        CHECK(a3->traj.target == hold3 && a4->traj.target == hold4);
        CHECK(s->correction_rad_s == 0.0f && s->timing.total_ms == 1u);
        CHECK(MotorApp_Move2006Pair(sign * 2.0f) == MOTORAPP_OK);
        CHECK(fabsf(a3->traj.target - a3->motor->angle_rad + sign * 2.0f * 6.28318530718f / 36.0f) < 1e-5f);
        CHECK(s->state == MOTORAPP_PAIR_MOVING && s->timing.status == 1u);
    } else if (strcmp(argv[1], "timing") == 0 || strcmp(argv[1], "timing_wrap") == 0) {
        float sign = strcmp(argv[1], "timing_wrap") == 0 ? -1.0f : 1.0f;
        uint32_t started, finished;
        if (sign < 0.0f) { tick = UINT32_MAX - 10u; cycle(-1); }
        started = tick;
        CHECK(MotorApp_Move2006Pair(sign * 250.0f) == MOTORAPP_OK);
        CHECK(s->timing.start_ms == started && s->timing.status == 1u);
        for (n = 0u; n < 5000u; n++) { cycle(-1); }
        CHECK(s->timing.total_ms == 5000u && s->timing.phase == 4u);
        CHECK(s->timing.accel_ms > 0u && s->timing.cruise_ms > 0u && s->timing.decel_ms > 0u);
        CHECK(s->timing.follow_ms > 0u && s->timing.planner_ms < 5000u);
        CHECK(s->timing.accel_ms + s->timing.cruise_ms + s->timing.decel_ms +
              s->timing.follow_ms == s->timing.total_ms);
        CHECK(s->timing.accel_ms + s->timing.cruise_ms + s->timing.decel_ms == s->timing.planner_ms);
        CHECK(s->timing.first_motion_ms[0] == UINT32_MAX && s->timing.reach_75_ms == UINT32_MAX);
        CHECK(s->timing.stopped_ms[0] == 5000u && s->timing.stopped_ms[1] == 5000u);
        place_pair(sign * 200.0f, sign * 200.0f); cycle(-1);
        CHECK(s->timing.first_motion_ms[0] == 5001u && s->timing.first_motion_ms[1] == 5001u);
        CHECK(s->timing.reach_75_ms == 5001u);
        place_pair(sign * 250.0f, sign * 200.0f);
        for (n = 0u; n < 10u; n++) { cycle(-1); }
        CHECK(s->timing.first_in_tol_ms[0] == 5002u && s->timing.first_in_tol_ms[1] == UINT32_MAX);
        place_pair(sign * 250.0f, sign * 250.0f); cycle(-1);
        CHECK(s->timing.first_in_tol_ms[1] == UINT32_MAX); /* losing side never reached original goal */
        CHECK(s->timing.status == 2u && s->timing.phase == 5u && s->timing.total_ms == 5002u);
        finished = s->timing.total_ms;
        place_pair(sign * 249.0f, sign * 249.0f);
        for (n = 0u; n < 100u; n++) { cycle(-1); }
        CHECK(s->state == MOTORAPP_PAIR_DONE && s->timing.total_ms == finished);
        place_pair(sign * 250.0f, sign * 250.0f); cycle(-1);
        CHECK(MotorApp_Move2006Pair(sign * 10.0f) == MOTORAPP_OK);
        CHECK(s->timing.total_ms == 0u && s->timing.status == 1u);
        CHECK(s->timing.first_motion_ms[0] == UINT32_MAX && s->timing.planner_ms == UINT32_MAX);
    } else if (strcmp(argv[1], "position_finish") == 0) {
        CHECK(MotorApp_Move2006Pair(0.3f) == MOTORAPP_OK);
        for (n = 0u; n < 1500u; n++) { cycle(-1); }
        CHECK(a3->traj.active == 0u && s->state == MOTORAPP_PAIR_MOVING);
        place_pair(0.3f, 0.3f);
        speed[3] = 100.0f; speed[4] = -100.0f;
        for (n = 0u; n < (unsigned int)(MOTORAPP_SETTLE_TIME_S * 1000.0f) + 2u; n++) { cycle(-1); }
        CHECK(s->state == MOTORAPP_PAIR_DONE); /* moving speed is not an arrival gate */
        CHECK(a3->state == AXIS_IDLE && a4->state == AXIS_IDLE);
    } else if (strcmp(argv[1], "early_position") == 0) {
        CHECK(MotorApp_Move2006Pair(50.19518086f) == MOTORAPP_OK);
        CHECK(a3->traj.active != 0u);
        place_pair(50.19518086f, 50.19518086f);
        speed[3] = 100.0f; speed[4] = -100.0f;
        cycle(-1);
        CHECK(s->state == MOTORAPP_PAIR_DONE);
        CHECK(a3->traj.active == 0u && a4->traj.active == 0u);
        CHECK(fabsf(s->reference_turns - 50.19518086f) < 0.001f);
    } else if (strcmp(argv[1], "terminal_speed") == 0) {
        CHECK(MotorApp_Move2006Pair(1.0f) == MOTORAPP_OK);
        for (n = 0u; n < 3000u; n++) { cycle(-1); }
        CHECK(a3->traj.active == 0u && s->state == MOTORAPP_PAIR_MOVING);
        CHECK(fabsf(a3->cascade.spd.setpoint) > MOTORPAIR_CFG_2006.sync_speed_max + 0.5f);
        CHECK(fabsf(a4->cascade.spd.setpoint) > MOTORPAIR_CFG_2006.sync_speed_max + 0.5f);
        CHECK(fabsf(a3->cascade.spd.setpoint) <= a3->cascade.pos.out_max);
    } else if (strcmp(argv[1], "startup_load") == 0) {
        unsigned int first_drive = 0u;
        CHECK(MotorApp_Move2006Pair(75.19518086f) == MOTORAPP_OK);
        /* User measured roughly 1000 raw command to overcome the initial load.
         * Stationary-feedback probe checks command buildup, not physical arrival. */
        for (n = 1u; n <= 500u; n++) {
            cycle(-1);
            if (first_drive == 0u && g_motor_2006_2.cmd <= -1000 && g_motor_2006_1.cmd >= 1000) {
                first_drive = n;
            }
        }
        printf("Stationary-feedback command reaches +/-1000 at %.3f s; at 0.5s: %d / %d\n",
               (double)first_drive / 1000.0, (int)g_motor_2006_2.cmd, (int)g_motor_2006_1.cmd);
        CHECK(first_drive != 0u && first_drive <= 500u);
    } else if (strcmp(argv[1], "concurrent_id5") == 0) {
        CHECK(MotorApp_Move2006Pair(75.0f) == MOTORAPP_OK);
        CHECK(MotorApp_MoveTurns(MotorApp_GetAxis(0u), -35.0f) == MOTORAPP_OK);
        for (n = 0u; n < 100u; n++) { cycle(-1); }
        CHECK(s->state == MOTORAPP_PAIR_MOVING);
        CHECK(MotorApp_GetAxis(0u)->traj.active != 0u);
        CHECK(MotorApp_GetAxis(0u)->motor->cmd < 0);
        CHECK(fabsf(MotorApp_GetAxis(0u)->traj.target + 35.0f * 6.28318530718f /
                    MotorApp_GetAxis(0u)->motor->model->gear_ratio) < 1e-5f);
    } else if (strcmp(argv[1], "rolling_start") == 0) {
        speed[4] = -6000.0f; speed[3] = 6000.0f; cycle(-1);
        CHECK(MotorApp_Move2006Pair(3.0f) == MOTORAPP_OK);
        cycle(-1);
        CHECK(fabsf(a3->cascade.spd.setpoint) <= a3->cascade.pos.out_max);
        CHECK(fabsf(a4->cascade.spd.setpoint) <= a4->cascade.pos.out_max);
    } else if (strcmp(argv[1], "zero_offset") == 0) {
        place_pair(2.0f, -3.0f); cycle(-1);
        CHECK(MotorApp_Move2006Pair(0.0f) == MOTORAPP_OK);
        for (n = 0u; n < 80u; n++) { cycle(-1); }
        CHECK(s->state == MOTORAPP_PAIR_DONE);
        CHECK(fabsf(s->actual_turns[0]) < 0.001f && fabsf(s->actual_turns[1]) < 0.001f);
        CHECK(MotorApp_Move2006Pair(-1.0f) == MOTORAPP_OK);
        CHECK(fabsf(a3->traj.target - a3->motor->angle_rad - 6.28318530718f / 36.0f) < 1e-5f);
        CHECK(fabsf(a4->traj.target - a4->motor->angle_rad + 6.28318530718f / 36.0f) < 1e-5f);
    } else if (strcmp(argv[1], "settle") == 0) {
        CHECK(MotorApp_Move2006Pair(0.2f) == MOTORAPP_OK);
        for (n = 0u; n < 1500u; n++) { cycle(-1); }
        CHECK(a3->traj.active == 0u && a4->traj.active == 0u);
        place_pair(0.2f, 0.0f);
        for (n = 0u; n < 80u; n++) { cycle(-1); }
        CHECK(a3->state == AXIS_IDLE && a4->state == AXIS_IDLE);
        place_pair(0.0f, 0.2f);
        for (n = 0u; n < 80u; n++) { cycle(-1); }
        CHECK(s->state == MOTORAPP_PAIR_DONE); /* any-side completion is latched */
        place_pair(0.2f, 0.2f);
        speed[3] = 100.0f;
        for (n = 0u; n < (unsigned int)(MOTORAPP_SETTLE_TIME_S * 1000.0f) + 2u; n++) { cycle(-1); }
        CHECK(s->state == MOTORAPP_PAIR_DONE);
        CHECK(a3->state == AXIS_IDLE && a4->state == AXIS_IDLE);
        speed[3] = 0.0f;
        place_pair(0.0f, 0.2f); cycle(-1);
        CHECK(s->state == MOTORAPP_PAIR_DONE); /* separate holding never restarts the old travel */
    } else if (strcmp(argv[1], "clock") == 0 || strcmp(argv[1], "blocked_side") == 0) {
        CHECK(MotorApp_Move2006Pair(75.0f) == MOTORAPP_OK);
        for (n = 0u; n < 5000u; n++) {
            cycle(-1);
            if (strcmp(argv[1], "blocked_side") == 0) { position[4] = -s->reference_turns; }
        }
        CHECK(fabsf(s->reference_turns - 75.0f) < 0.001f);
        CHECK(a3->traj.active == 0u && s->time_scale == 1.0f);
        CHECK(s->state == (strcmp(argv[1], "blocked_side") == 0 ? MOTORAPP_PAIR_DONE : MOTORAPP_PAIR_MOVING));
        CHECK(s->fault == MOTORAPP_PAIR_FAULT_NONE);
    } else if (strcmp(argv[1], "no_timeout") == 0) {
        CHECK(MotorApp_Move2006Pair(75.0f) == MOTORAPP_OK);
        for (n = 0u; n < 101000u; n++) { cycle(-1); }
        CHECK(s->state == MOTORAPP_PAIR_MOVING && s->fault == MOTORAPP_PAIR_FAULT_NONE);
    } else if (strcmp(argv[1], "fault") == 0 || strcmp(argv[1], "fault_id4") == 0) {
        unsigned int missed = strcmp(argv[1], "fault_id4") == 0 ? 1u : 0u;
        int16_t previous_cmd[2] = {0, 0};
        uint32_t saved_count;
        CHECK(MotorApp_Move2006Pair(2.0f) == MOTORAPP_OK);
        for (n = 0u; n < 55u; n++) {
            if (n == 50u) { previous_cmd[0] = g_motor_2006_2.cmd; previous_cmd[1] = g_motor_2006_1.cmd; }
            cycle(missed == 0u ? 4 : 3);
        }
        CHECK(s->state == MOTORAPP_PAIR_FAULT && s->fault == MOTORAPP_PAIR_FAULT_OFFLINE);
        CHECK(s->fault_motor_id == (missed == 0u ? 3u : 4u));
        CHECK(s->fault_tick_ms == tick - 4u);
        CHECK(s->timing.status == 3u && s->timing.phase == 6u);
        CHECK(s->timing.total_ms == 51u);
        CHECK(s->fault_rx_age_ms[missed] == 51u);
        CHECK(s->fault_rx_age_ms[1u - missed] == 0u);
        CHECK(s->fault_command[0] == previous_cmd[0] && s->fault_command[1] == previous_cmd[1]);
        saved_count = s->fault_rx_count[missed];
        for (n = 0u; n < 100u; n++) { cycle(-1); }
        CHECK(s->fault_rx_age_ms[missed] == 51u && s->fault_rx_count[missed] == saved_count);
        CHECK(s->timing.total_ms == 51u); /* fault timing freezes with snapshot */
        CHECK(a3->state == AXIS_FAULT && a4->state == AXIS_FAULT);
        CHECK(g_motor_2006_1.cmd == 0 && g_motor_2006_2.cmd == 0);
        CHECK(MotorApp_Move2006Pair(1.0f) == MOTORAPP_ERR_FAULT);
    } else if (strcmp(argv[1], "sync_reporting") == 0) {
        CHECK(MotorApp_Move2006Pair(75.0f) == MOTORAPP_OK);
        place_pair(7.0f, 0.0f); cycle(-1);
        CHECK(s->state == MOTORAPP_PAIR_MOVING && s->fault == MOTORAPP_PAIR_FAULT_NONE);
        CHECK(s->sync_error_turns > 6.99f && s->correction_rad_s > 0.0f);
    } else if (strcmp(argv[1], "correction") == 0) {
        CHECK(MotorApp_Move2006Pair(10.0f) == MOTORAPP_OK);
        place_pair(0.4f, 0.0f);
        for (n = 0u; n < 100u; n++) { cycle(-1); }
        CHECK(s->sync_error_turns > 0.39f);
        CHECK(s->correction_rad_s > 0.0f);
        CHECK(-a3->cascade.spd.setpoint < a4->cascade.spd.setpoint);
        CHECK(-a3->cascade.spd.setpoint >= 0.0f);
    } else {
        float sign = strcmp(argv[1], "reverse") == 0 ? -1.0f : 1.0f;
        float previous3 = 0.0f, previous4 = 0.0f;
        CHECK(MotorApp_Move2006Pair(sign * 3.0f) == MOTORAPP_OK);
        /* Kinematic speed plant with unequal response; not a measured motor model. */
        for (n = 0u; n < 12000u && s->state == MOTORAPP_PAIR_MOVING; n++) {
            float cmd3, cmd4;
            cycle(-1);
            cmd3 = a3->cascade.spd.setpoint;
            cmd4 = a4->cascade.spd.setpoint;
            if (s->state == MOTORAPP_PAIR_MOVING) {
                CHECK(fabsf(cmd3 - previous3) <= MOTORPAIR_CFG_2006.command_a_max * 0.001f + 1e-4f);
                CHECK(fabsf(cmd4 - previous4) <= MOTORPAIR_CFG_2006.command_a_max * 0.001f + 1e-4f);
            }
            previous3 = cmd3; previous4 = cmd4;
            speed[4] += 0.08f * (cmd3 * 36.0f * 60.0f / 6.28318530718f - speed[4]);
            speed[3] += 0.03f * (cmd4 * 36.0f * 60.0f / 6.28318530718f - speed[3]);
            position[4] += speed[4] / 60000.0f;
            position[3] += speed[3] / 60000.0f;
        }
        CHECK(s->state == MOTORAPP_PAIR_DONE && n < 12000u);
        CHECK(fabsf(s->actual_turns[0] - sign * 3.0f) <= MOTORAPP_POSITION_TOLERANCE_ROTOR_TURNS ||
              fabsf(s->actual_turns[1] - sign * 3.0f) <= MOTORAPP_POSITION_TOLERANCE_ROTOR_TURNS);
    }
    printf("2006 pair %s: %u failures\n", argv[1], failures);
    return failures == 0u ? 0 : 1;
}
