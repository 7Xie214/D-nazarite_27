/* Real APP, motor decoder, calibration, PID and trajectory implementation.
 * Only HAL time, CAN transport and the unrelated encoder observer are stubs. */
#include "ChassisApp.h"
#include "PID_Config.h"
#include "fdcan.h"
#include "My_CAN.h"
#include <math.h>
#include <float.h>
#include <stdio.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
uint32_t test_irq_mask;
static uint32_t tick;
static unsigned int checks, failures, frames, wrong_frames;
static int16_t packet[2][2][4];
static unsigned int frame_seen[2][2];
static DjiMotor_t *const drive[2] = {&g_motor_3508_4, &g_motor_3508_5};
static DjiMotor_t *const steer[2] = {&g_motor_6020_1, &g_motor_6020_4};
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("FAIL %d: %s\n", __LINE__, #c); } } while (0)
#define CLOSE(a,b,e) CHECK(fabsf((a)-(b)) <= (e))
#define NEAR(a,b) CLOSE((a),(b),0.0002f)
static const float tau = 6.2831853071795864769f;

uint32_t HAL_GetTick(void) { return tick; }
uint32_t osKernelGetTickCount(void) { return tick; }
void Test_EncoderOnCanFrame(FDCAN_HandleTypeDef *h, uint16_t id, const uint8_t *d, uint8_t n)
{ (void)h; (void)id; (void)d; (void)n; }
HAL_StatusTypeDef BSP_CAN_Send(FDCAN_HandleTypeDef *h, uint16_t id, const uint8_t *d, uint8_t n)
{
    unsigned int i, bus, group;
    if (((h != &hfdcan1) && (h != &hfdcan2)) || ((id != 0x200u) && (id != 0x1ffu)) || (n != 8u)) {
        wrong_frames++; return HAL_ERROR;
    }
    bus = h == &hfdcan1 ? 0u : 1u;
    group = id == 0x200u ? 0u : 1u;
    for (i = 0u; i < 4u; i++) {
        uint16_t v = (uint16_t)((uint16_t)d[2u*i] << 8u) | d[2u*i+1u];
        packet[bus][group][i] = (int16_t)v;
    }
    frame_seen[bus][group]++;
    frames++;
    return HAL_OK;
}
static void feed(DjiMotor_t *m, uint16_t raw, int16_t rpm)
{
    uint16_t speed = (uint16_t)rpm;
    uint8_t d[8] = {0u};
    d[0] = (uint8_t)(raw >> 8u); d[1] = (uint8_t)raw;
    d[2] = (uint8_t)(speed >> 8u); d[3] = (uint8_t)speed;
    DjiMotor_OnCanFrame(m->hfdcan, (uint16_t)(m->model->feedback_base + m->id), d, 8u);
}
static void refresh(void)
{
    unsigned int i;
    for (i = 0u; i < 2u; i++) {
        feed(drive[i], drive[i]->raw_angle, drive[i]->raw_speed);
        feed(steer[i], steer[i]->raw_angle, steer[i]->raw_speed);
    }
}
static void clear_frames(void)
{
    frames = 0u; wrong_frames = 0u;
    memset(frame_seen, 0, sizeof(frame_seen));
    memset(packet, 0, sizeof(packet));
}
static void update(float dt)
{
    clear_frames();
    ChassisApp_UpdateFrontPair(dt);
}
static void reset_feedback(DjiMotor_t *m)
{
    FDCAN_HandleTypeDef *bus = m->hfdcan;
    const DjiMotor_Model_t *model = m->model;
    uint8_t id = m->id;
    memset(m, 0, sizeof(*m));
    m->hfdcan = bus; m->model = model; m->id = id;
}
static void restart(int ready)
{
    unsigned int i;
    tick = 100u;
    for (i = 0u; i < 2u; i++) { reset_feedback(drive[i]); reset_feedback(steer[i]); }
    Chassis_Init(); ChassisApp_Init();
    if (ready != 0) {
        for (i = 0u; i < 2u; i++) {
            feed(drive[i], (uint16_t)(1000u + i * 1000u), 0);
            feed(steer[i], g_swerve_modules[i].rudder_direction.zero_offset, 0);
        }
        update(0.001f);
    }
    clear_frames();
}
/* Accumulate genuine wrapped CAN encoder feedback with each delta < half a
 * revolution. No application status or engineering feedback is fabricated. */
static void position(unsigned int i, float rotor_turns, int16_t rpm)
{
    DjiMotor_t *m = drive[i];
    int32_t target = (int32_t)lroundf(rotor_turns * 8192.0f);
    while (m->total_counts != target) {
        int32_t delta = target - m->total_counts;
        int32_t raw;
        if (delta > 1024) { delta = 1024; }
        if (delta < -1024) { delta = -1024; }
        raw = (int32_t)m->raw_angle + delta;
        raw %= 8192;
        if (raw < 0) { raw += 8192; }
        feed(m, (uint16_t)raw, rpm);
        DjiMotor_Update(m);
    }
    feed(m, m->raw_angle, rpm);
}
static void check_frames(void)
{
    unsigned int b, g, s;
    CHECK(frames == 4u && wrong_frames == 0u);
    for (b = 0u; b < 2u; b++) {
        for (g = 0u; g < 2u; g++) {
            CHECK(frame_seen[b][g] == 1u);
            for (s = 0u; s < 4u; s++) {
                int16_t value = packet[b][g][s];
                if (g == 0u) {
                    if (((b == 0u) && (s != 0u)) || ((b == 1u) && (s != 3u))) { CHECK(value == 0); }
                } else if (((b == 0u) && (s >= 2u)) || ((b == 1u) && (s < 2u))) {
                    CHECK(value == 0);
                }
                CHECK(value <= (g == 0u ? 16384 : 30000));
                CHECK(value >= (g == 0u ? -16384 : -30000));
            }
        }
    }
    CHECK(packet[0][0][0] == drive[0]->cmd);
    CHECK(packet[1][0][3] == drive[1]->cmd);
    CHECK(packet[0][1][0] == steer[0]->cmd);
    CHECK(packet[1][1][3] == steer[1]->cmd);
    CHECK(packet[0][1][1] == g_motor_6020_2.cmd);
    CHECK(packet[1][1][2] == g_motor_6020_3.cmd);
}
static void test_ready_atomic(void)
{
    unsigned int missing, i;
    restart(0); update(0.001f);
    CHECK(drive[0]->cmd == 0 && drive[1]->cmd == 0);
    CHECK(steer[0]->cmd == 0 && steer[1]->cmd == 0);
    CHECK(ChassisApp_MoveFrontPairTurns(10.0f, -10.0f) == CHASSIS_FRONT_START_NOT_READY);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_HOLD);
    check_frames();
    for (missing = 0u; missing < 4u; missing++) {
        restart(0);
        for (i = 0u; i < 2u; i++) {
            if (missing != i) { feed(drive[i], 123u, 0); }
            if (missing != i + 2u) { feed(steer[i], g_swerve_modules[i].rudder_direction.zero_offset, 0); }
        }
        update(0.001f);
        CHECK(ChassisApp_MoveFrontPairTurns(10.0f, -10.0f) != CHASSIS_FRONT_START_OK);
        CHECK(g_chassis_front_state != CHASSIS_FRONT_MOVING);
        CHECK(g_chassis_front_pair.planner_active == 0u);
    }
    restart(1);
    CHECK(g_chassis_app_status == CHASSIS_APP_READY);
    CHECK(g_motor_3508_6.rx_count == 0u && g_motor_3508_7.rx_count == 0u);
    CHECK(g_motor_2006_1.rx_count == 0u);
    CHECK(ChassisApp_MoveFrontPairTurns(0.0f, 0.0f) == CHASSIS_FRONT_START_BAD_CONFIG);
    CHECK(ChassisApp_MoveFrontPairTurns(10.0f, -9.0f) == CHASSIS_FRONT_START_BAD_CONFIG);
    CHECK(ChassisApp_MoveFrontPairTurns(NAN, 10.0f) == CHASSIS_FRONT_START_BAD_CONFIG);
    CHECK(ChassisApp_MoveFrontPairTurns(INFINITY, 10.0f) == CHASSIS_FRONT_START_BAD_CONFIG);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_HOLD);
    CHECK(g_chassis_front_pair.move_sequence == 0u);
    CHECK(ChassisApp_MoveFrontPairTurns(17.402660f, -17.4026606f) == CHASSIS_FRONT_START_OK);
    CHECK(g_chassis_front_pair.move_sequence == 1u);
    CHECK(fabsf(g_chassis_front_pair.error_turns[0] - 17.402660f) < 0.00001f);
    CHECK(fabsf(g_chassis_front_pair.error_turns[1] + 17.4026606f) < 0.00001f);
    restart(1);
    position(0u, 3.0f, 0); position(1u, -2.0f, 0); update(0.001f);
    CHECK(ChassisApp_MoveFrontPairTurns(10.0f, -10.0f) == CHASSIS_FRONT_START_OK);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_MOVING);
    CHECK(ChassisApp_MoveFrontPairTurns(15.0f, 15.0f) == CHASSIS_FRONT_START_BUSY);
    NEAR(g_chassis_front_pair.reference_turns, 0.0f);
    NEAR(g_chassis_front_pair.error_turns[0], 10.0f);
    NEAR(g_chassis_front_pair.error_turns[1], -10.0f);
    CHECK(g_chassis_front_pair.start_ms == tick);
    tick += 51u;
    CHECK(ChassisApp_MoveFrontPairTurns(10.0f, -10.0f) == CHASSIS_FRONT_START_BUSY);
    update(0.001f);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_FAULT);

    /* Ready yesterday is insufficient: preflight checks fresh feedback before
     * either planner starts, without requiring another control update. */
    for (missing = 0u; missing < 4u; missing++) {
        restart(1); tick += 51u;
        for (i = 0u; i < 2u; i++) {
            if (missing != i) { feed(drive[i],drive[i]->raw_angle,0); }
            if (missing != i+2u) { feed(steer[i],steer[i]->raw_angle,0); }
        }
        CHECK(ChassisApp_MoveFrontPairTurns(10.0f,-10.0f) == CHASSIS_FRONT_START_NOT_READY);
        CHECK(g_chassis_front_state == CHASSIS_FRONT_HOLD);
        CHECK(g_chassis_front_pair.planner_active == 0u);
        CHECK(g_chassis_front_axes[0].traj.active == 0u && g_chassis_front_axes[1].traj.active == 0u);
        refresh(); update(0.001f);
        CHECK(ChassisApp_MoveFrontPairTurns(10.0f,-10.0f) == CHASSIS_FRONT_START_OK);
    }

    /* Already accumulated nonzero drives seed their measured positions, while
     * the two steer references remain calibrated absolute zero. */
    restart(0);
    for (i = 0u; i < 2u; i++) {
        feed(drive[i],1234u,0); DjiMotor_Update(drive[i]);
        position(i,i == 0u ? 4.0f : -3.0f,0);
        feed(steer[i],(uint16_t)((g_swerve_modules[i].rudder_direction.zero_offset+16u)%8192u),0);
    }
    update(0.001f);
    for (i = 0u; i < 2u; i++) {
        NEAR(g_chassis_front_axes[i].traj.target,
             (float)g_swerve_modules[i].steering_gear.encoder_direction*drive[i]->angle_rad);
        CHECK(drive[i]->cmd == 0);
        CHECK(steer[i]->cmd != 0);
        NEAR(g_swerve_modules[i].target_angle_rad,0.0f);
    }
    check_frames();
}
static void test_no_drive_control_before_request(void)
{
    unsigned int i, n;
    float origin[2];
    restart(1);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_HOLD);
    CHECK(ChassisApp_MoveFrontPairTurns(10.0f, -9.0f) == CHASSIS_FRONT_START_BAD_CONFIG);
    for (n = 0u; n < 20u; n++) {
        tick++;
        /* External mechanism rolls both wheels before third-stair permission. */
        position(0u, 1.0f + (float)n * 0.125f, 300);
        position(1u, -2.0f - (float)n * 0.125f, -400);
        for (i = 0u; i < 2u; i++) {
            feed(steer[i], (uint16_t)((g_swerve_modules[i].rudder_direction.zero_offset + 16u) % 8192u), 0);
        }
        update(0.001f);
        CHECK(g_chassis_front_state == CHASSIS_FRONT_HOLD);
        CHECK(drive[0]->cmd == 0 && drive[1]->cmd == 0);
        CHECK(packet[0][0][0] == 0 && packet[1][0][3] == 0);
        for (i = 0u; i < 2u; i++) {
            NEAR(g_chassis_front_axes[i].cascade.pos.integral, 0.0f);
            NEAR(g_chassis_front_axes[i].cascade.spd.integral, 0.0f);
            NEAR(g_chassis_front_pair.speed_command_rpm[i], 0.0f);
            CHECK(steer[i]->cmd != 0); /* Steering remains active. */
            NEAR(g_swerve_modules[i].target_angle_rad, 0.0f);
        }
    }
    for (i = 0u; i < 2u; i++) { origin[i] = drive[i]->angle_rad; }
    CHECK(ChassisApp_MoveFrontPairTurns(12.2969213f, -12.2969213f) == CHASSIS_FRONT_START_OK);
    for (i = 0u; i < 2u; i++) {
        float sign = i == 0u ? 1.0f : -1.0f;
        float encoder = (float)g_swerve_modules[i].steering_gear.encoder_direction;
        CLOSE(g_chassis_front_axes[i].traj.target,
              encoder * (origin[i] + sign * 12.2969213f * tau / DJI_MODEL_3508.gear_ratio), 0.0002f);
    }
}
static void test_signs_shared_reference(void)
{
    unsigned int sign_case, n, i;
    const float signs[4][2] = {{1.0f,-1.0f},{-1.0f,1.0f},{1.0f,1.0f},{-1.0f,-1.0f}};
    for (sign_case = 0u; sign_case < 4u; sign_case++) {
        float previous[2] = {0.0f,0.0f};
        restart(1);
        CHECK(ChassisApp_MoveFrontPairTurns(signs[sign_case][0]*10.0f, signs[sign_case][1]*10.0f) == CHASSIS_FRONT_START_OK);
        for (n = 0u; n < 100u; n++) {
            tick++; refresh(); update(0.001f);
            CHECK(g_chassis_front_state == CHASSIS_FRONT_MOVING);
            CHECK(g_chassis_front_pair.planner_active != 0u);
            for (i = 0u; i < 2u; i++) {
                float raw_command = g_chassis_front_pair.speed_command_rpm[i];
                float max_change = MOTORPAIR_CFG_CHASSIS_FRONT.command_a_max * 0.001f * drive[i]->model->gear_ratio * 60.0f / tau;
                CHECK(signs[sign_case][i]*raw_command > 0.0f);
                CHECK(fabsf(raw_command-previous[i]) <= max_change + 0.002f);
                previous[i] = raw_command;
                CLOSE(g_chassis_front_axes[i].traj.pos_ref,
                      (float)g_swerve_modules[i].steering_gear.encoder_direction*signs[sign_case][i]*
                      g_chassis_front_pair.reference_turns*tau/drive[i]->model->gear_ratio,0.0002f);
            }
            NEAR(fabsf(g_chassis_front_pair.speed_command_rpm[0]), fabsf(g_chassis_front_pair.speed_command_rpm[1]));
        }
        CHECK(g_chassis_front_pair.reference_turns > 0.0f && g_chassis_front_pair.reference_turns < 10.0f);
        CHECK(signs[sign_case][0]*(float)drive[0]->cmd > 0.0f);
        CHECK(signs[sign_case][1]*(float)drive[1]->cmd > 0.0f);
        NEAR(g_chassis_front_pair.sync_error_turns, 0.0f);
        CHECK(g_chassis_front_pair.elapsed_ms == 100u);
        check_frames();
        position(0u, signs[sign_case][0]*2.0f, 0); position(1u, signs[sign_case][1]*1.0f, 0);
        refresh(); update(0.001f);
        NEAR(g_chassis_front_pair.actual_turns[0], signs[sign_case][0]*2.0f);
        NEAR(g_chassis_front_pair.actual_turns[1], signs[sign_case][1]*1.0f);
        NEAR(g_chassis_front_pair.error_turns[0], signs[sign_case][0]*8.0f);
        NEAR(g_chassis_front_pair.error_turns[1], signs[sign_case][1]*9.0f);
        NEAR(g_chassis_front_pair.sync_error_turns, 1.0f);
        CHECK(g_chassis_front_pair.correction_rad_s > 0.0f);
        CHECK(g_chassis_front_pair.correction_rad_s <= MOTORPAIR_CFG_CHASSIS_FRONT.sync_speed_max);
    }
}
static void test_sync_and_steer(void)
{
    unsigned int n;
    float baseline[2] = {0.0f,0.0f}, leader[2] = {0.0f,0.0f}, lagger[2] = {0.0f,0.0f};
    /* Identical physical positions in three independent runs isolate sync sign. */
    unsigned int scenario;
    for (scenario = 0u; scenario < 3u; scenario++) {
        restart(1);
        CHECK(ChassisApp_MoveFrontPairTurns(20.0f, -20.0f) == CHASSIS_FRONT_START_OK);
        if (scenario == 1u) { position(0u, 1.0f, 0); }
        if (scenario == 2u) { position(1u, -1.0f, 0); }
        /* Let command slew reach the unsaturated requested speeds. */
        for (n = 0u; n < 150u; n++) { tick++; refresh(); update(0.001f); }
        if (scenario == 0u) { baseline[0] = g_chassis_front_pair.speed_command_rpm[0]; baseline[1] = -g_chassis_front_pair.speed_command_rpm[1]; }
        if (scenario == 1u) { leader[0] = g_chassis_front_pair.speed_command_rpm[0]; leader[1] = -g_chassis_front_pair.speed_command_rpm[1]; }
        if (scenario == 2u) { lagger[0] = g_chassis_front_pair.speed_command_rpm[0]; lagger[1] = -g_chassis_front_pair.speed_command_rpm[1]; }
    }
    CHECK(leader[0] < baseline[0]); CHECK(leader[1] > baseline[1]);
    CHECK(lagger[0] > baseline[0]); CHECK(lagger[1] < baseline[1]);
    feed(steer[0], (uint16_t)((g_swerve_modules[0].rudder_direction.zero_offset+16u)%8192u), 0);
    feed(steer[1], (uint16_t)((g_swerve_modules[1].rudder_direction.zero_offset+16u)%8192u), 0);
    update(0.001f);
    NEAR(g_swerve_modules[0].target_angle_rad,0.0f);
    NEAR(g_swerve_modules[1].target_angle_rad,0.0f);
    CHECK(steer[0]->cmd != 0 && steer[1]->cmd != 0);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_MOVING);
    check_frames();
}
static void test_winners_and_hold(void)
{
    unsigned int winner, sign_case;
    const float signs[2][2] = {{1.0f,-1.0f},{-1.0f,1.0f}};
    for (winner = 0u; winner < 2u; winner++) {
        for (sign_case = 0u; sign_case < 2u; sign_case++) {
            float held[2];
            unsigned int i;
            restart(1); position(0u,3.0f,0); position(1u,-2.0f,0); update(0.001f);
            CHECK(ChassisApp_MoveFrontPairTurns(signs[sign_case][0]*10.0f,signs[sign_case][1]*10.0f) == CHASSIS_FRONT_START_OK);
            position(winner,(winner == 0u ? 3.0f : -2.0f)+signs[sign_case][winner]*9.85f,1000);
            refresh(); update(0.001f);
            CHECK(g_chassis_front_state == CHASSIS_FRONT_MOVING); /* Outside0.1 rotor turns. */
            for (i = 0u; i < 2u; i++) {
                float origin = i == 0u ? 3.0f : -2.0f;
                position(i, origin+signs[sign_case][i]*(i == winner ? 9.95f : 2.0f), 1000);
                g_chassis_front_axes[i].cascade.pos.integral = 0.5f;
                g_chassis_front_axes[i].cascade.spd.integral = 0.5f;
            }
            tick++; refresh(); update(0.001f);
            CHECK(g_chassis_front_state == CHASSIS_FRONT_DONE);
            CHECK(g_chassis_front_pair.finish_motor_id == drive[winner]->id);
            CHECK(g_chassis_front_pair.planner_active == 0u);
            NEAR(g_chassis_front_pair.correction_rad_s,0.0f);
            for (i = 0u; i < 2u; i++) {
                held[i] = (float)g_swerve_modules[i].steering_gear.encoder_direction*drive[i]->angle_rad;
                NEAR(g_chassis_front_axes[i].traj.target,held[i]);
                NEAR(g_chassis_front_axes[i].traj.pos_ref,held[i]);
                NEAR(g_chassis_front_axes[i].cascade.pos.integral,0.0f);
                /* Completion clears the old 0.5 accumulator; this very cycle's
                 * independent hold may integrate one fresh braking sample. */
                CHECK(fabsf(g_chassis_front_axes[i].cascade.spd.integral) <=
                      fabsf(drive[i]->speed_rad_s)*0.001f+0.00001f);
                CHECK(g_chassis_front_axes[i].traj.active == 0u);
                NEAR(g_chassis_front_pair.speed_command_rpm[i],0.0f);
            }
            CHECK(fabsf(g_chassis_front_pair.error_turns[1u-winner]) > 7.9f);
            for (i = 0u; i < 2u; i++) { position(i,(float)drive[i]->total_counts/8192.0f+0.5f,0); }
            tick++; refresh(); update(0.001f);
            CHECK(g_chassis_front_state == CHASSIS_FRONT_DONE);
            NEAR(g_chassis_front_pair.correction_rad_s,0.0f);
            NEAR(g_chassis_front_axes[0].traj.target,held[0]);
            NEAR(g_chassis_front_axes[1].traj.target,held[1]);
            CHECK(ChassisApp_MoveFrontPairTurns(-5.0f,5.0f) == CHASSIS_FRONT_START_OK);
            NEAR(g_chassis_front_pair.actual_turns[0],0.0f);
            NEAR(g_chassis_front_pair.actual_turns[1],0.0f);
            NEAR(g_chassis_front_pair.error_turns[0],-5.0f);
            NEAR(g_chassis_front_pair.error_turns[1],5.0f);
            CHECK(g_chassis_front_pair.finish_motor_id == 0u);
            for (i = 0u; i < 2u; i++) {
                float encoder = (float)g_swerve_modules[i].steering_gear.encoder_direction;
                NEAR(g_chassis_front_axes[i].traj.target,
                     encoder*(drive[i]->angle_rad+(i == 0u ? -5.0f : 5.0f)*tau/drive[i]->model->gear_ratio));
            }
        }
    }
}
static void test_faults_and_bounds(void)
{
    unsigned int missing, i;
    for (missing = 0u; missing < 4u; missing++) {
        restart(1);
        CHECK(ChassisApp_MoveFrontPairTurns(20.0f,-20.0f) == CHASSIS_FRONT_START_OK);
        tick += 51u;
        for (i = 0u; i < 2u; i++) {
            if (missing != i) { feed(drive[i],drive[i]->raw_angle,0); }
            if (missing != i+2u) { feed(steer[i],steer[i]->raw_angle,0); }
        }
        update(0.001f);
        CHECK(g_chassis_front_state == CHASSIS_FRONT_FAULT);
        CHECK(drive[0]->cmd == 0 && drive[1]->cmd == 0);
        CHECK(g_chassis_front_pair.planner_active == 0u);
        CHECK(g_chassis_front_pair.fault_motor_id == (missing < 2u ? drive[missing]->id : steer[missing-2u]->id));
        refresh(); update(0.001f);
        CHECK(g_chassis_front_state == CHASSIS_FRONT_FAULT);
        CHECK(g_chassis_app_status == CHASSIS_APP_FAULT_FEEDBACK);
        CHECK(drive[0]->cmd == 0 && drive[1]->cmd == 0);
        CHECK(ChassisApp_MoveFrontPairTurns(20.0f,-20.0f) != CHASSIS_FRONT_START_OK);
        feed(steer[0],(uint16_t)((g_swerve_modules[0].rudder_direction.zero_offset+16u)%8192u),0);
        feed(steer[1],(uint16_t)((g_swerve_modules[1].rudder_direction.zero_offset+16u)%8192u),0);
        update(0.001f);
        CHECK(steer[0]->cmd != 0 && steer[1]->cmd != 0);
        CHECK(drive[0]->cmd == 0 && drive[1]->cmd == 0);
        check_frames();
    }
    restart(1);
    CHECK(ChassisApp_MoveFrontPairTurns(100.0f,-100.0f) == CHASSIS_FRONT_START_OK);
    feed(drive[0],drive[0]->raw_angle,INT16_MIN); feed(drive[1],drive[1]->raw_angle,INT16_MAX);
    for (i = 0u; i < 2u; i++) {
        g_chassis_front_axes[i].cascade.spd.Kp = 1.0e9f;
        g_chassis_front_axes[i].cascade.spd.out_max = 1.0e9f;
    }
    update(0.001f);
    CHECK(drive[0]->cmd == 16384); CHECK(drive[1]->cmd == -16384);
    check_frames();
    update(NAN);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_FAULT);
    CHECK(drive[0]->cmd == 0 && drive[1]->cmd == 0);
}
static void test_rear_ownership_and_done_dropout(void)
{
    unsigned int i, missing;
    DjiMotor_t *const rear[4] = {&g_motor_3508_6,&g_motor_3508_7,&g_motor_6020_2,&g_motor_6020_3};
    restart(1);
    for (i = 0u; i < 4u; i++) { feed(rear[i],123u,0); rear[i]->cmd = (int16_t)(1000+(int)i); }
    feed(rear[2], (uint16_t)((g_swerve_modules[2].rudder_direction.zero_offset + 64u) % 8192u), 0);
    feed(rear[3], (uint16_t)((g_swerve_modules[3].rudder_direction.zero_offset + 8128u) % 8192u), 0);
    update(0.001f);
    CHECK(rear[0]->cmd == 0 && rear[1]->cmd == 0);
    CHECK(rear[2]->cmd != 0 && rear[3]->cmd == 0);
    check_frames();
    CHECK(ChassisApp_MoveFrontPairTurns(10.0f,-10.0f) == CHASSIS_FRONT_START_OK);
    for (i = 0u; i < 4u; i++) { rear[i]->cmd = -2000; }
    update(0.001f);
    CHECK(rear[0]->cmd == 0 && rear[1]->cmd == 0);
    CHECK(rear[2]->cmd != 0 && rear[3]->cmd == 0);
    check_frames();
    /* Rear dropout stops only rear steering; the accepted front pair continues. */
    tick += 51u; refresh(); update(0.001f);
    CHECK(rear[2]->cmd == 0 && rear[3]->cmd == 0);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_MOVING);
    CHECK(g_chassis_app_status == CHASSIS_APP_READY);
    for (missing = 0u; missing < 2u; missing++) {
        restart(1);
        CHECK(ChassisApp_MoveFrontPairTurns(10.0f,-10.0f) == CHASSIS_FRONT_START_OK);
        position(0u,10.0f,0); refresh(); update(0.001f);
        CHECK(g_chassis_front_state == CHASSIS_FRONT_DONE);
        tick += 51u;
        for (i = 0u; i < 2u; i++) {
            feed(drive[i],drive[i]->raw_angle,0);
            if (i != missing) { feed(steer[i],steer[i]->raw_angle,0); }
        }
        update(0.001f);
        CHECK(g_chassis_front_state == CHASSIS_FRONT_FAULT);
        CHECK(g_chassis_app_status == CHASSIS_APP_FAULT_FEEDBACK);
        CHECK(drive[0]->cmd == 0 && drive[1]->cmd == 0);
        CHECK(g_chassis_front_pair.fault_motor_id == steer[missing]->id);
        refresh(); update(0.001f);
        CHECK(g_chassis_front_state == CHASSIS_FRONT_FAULT);
        CHECK(g_chassis_app_status == CHASSIS_APP_FAULT_FEEDBACK);
    }
}
static void test_numeric_preflight(void)
{
    Traj_t before[2];
    ChassisFront_StartResult_t result;
    unsigned int i;
    restart(1);
    /* Fault injection at the APP input boundary: finite engineering feedback
     * can still overflow a signed endpoint, so stage both before commit. */
    drive[0]->angle_rad = FLT_MAX;
    for (i = 0u; i < 2u; i++) { before[i] = g_chassis_front_axes[i].traj; }
    CHECK(ChassisApp_MoveFrontPairTurns(FLT_MAX,-FLT_MAX) == CHASSIS_FRONT_START_BAD_CONFIG);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_HOLD);
    for (i = 0u; i < 2u; i++) {
        CHECK(memcmp(&before[i],&g_chassis_front_axes[i].traj,sizeof(Traj_t)) == 0);
    }
    CHECK(g_chassis_front_pair.planner_active == 0u);
    restart(1);
    result = ChassisApp_MoveFrontPairTurns(FLT_MAX,-FLT_MAX);
    CHECK(result == CHASSIS_FRONT_START_BAD_CONFIG || result == CHASSIS_FRONT_START_OK);
    if (result == CHASSIS_FRONT_START_OK) {
        for (i = 0u; i < 2u; i++) { CHECK(isfinite(g_chassis_front_axes[i].traj.target)); }
        update(0.001f);
        CHECK(isfinite(g_chassis_front_pair.reference_turns));
    }
}
static void test_no_reference_or_time_completion(void)
{
    unsigned int n;
    restart(1);
    CHECK(ChassisApp_MoveFrontPairTurns(10.0f,-10.0f) == CHASSIS_FRONT_START_OK);
    for (n = 0u; n < 120000u; n++) {
        tick++; refresh(); update(0.001f);
    }
    CHECK(g_chassis_front_pair.planner_active == 0u);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_MOVING);
    CHECK(g_chassis_front_pair.elapsed_ms == 120000u);
    NEAR(g_chassis_front_pair.reference_turns,10.0f);
    NEAR(g_chassis_front_pair.actual_turns[0],0.0f);
    NEAR(g_chassis_front_pair.actual_turns[1],0.0f);
    CHECK(drive[0]->cmd > 0 && drive[1]->cmd < 0);
    check_frames();
}
static void test_pause_at_fraction(void)
{
    unsigned int n, i;
    uint32_t sequence;
    restart(1);
    CHECK(ChassisApp_MoveFrontPairTurnsWithPause(10.0f,-10.0f,NAN) == CHASSIS_FRONT_START_BAD_CONFIG);
    CHECK(ChassisApp_MoveFrontPairTurnsWithPause(10.0f,-10.0f,1.0f) == CHASSIS_FRONT_START_BAD_CONFIG);
    CHECK(ChassisApp_MoveFrontPairTurnsWithPause(10.0f,-10.0f,0.4f) == CHASSIS_FRONT_START_OK);
    sequence = g_chassis_front_pair.move_sequence;
    CHECK(ChassisApp_ResumeFrontPair() == 0);
    for (n = 0u; n < 1600u; n++) {
        float reference = g_chassis_front_pair.reference_turns;
        tick++;
        for (i = 0u; i < 2u; i++) { position(i, i == 0u ? reference : -reference, 0); }
        refresh(); update(0.001f);
    }
    CHECK(g_chassis_front_state == CHASSIS_FRONT_MOVING);
    CHECK(g_chassis_front_pair.paused != 0u && g_chassis_front_pair.pause_ms != UINT32_MAX);
    CLOSE(g_chassis_front_pair.reference_turns,4.0f,0.001f);
    CLOSE(g_chassis_front_pair.actual_turns[0],4.0f,0.001f);
    CLOSE(g_chassis_front_pair.actual_turns[1],-4.0f,0.001f);
    CHECK(g_chassis_front_pair.resume_ms == UINT32_MAX);
    /* Even reaching the FINAL endpoint on one wheel cannot bypass the pause. */
    position(0u,10.0f,0); refresh(); update(0.001f);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_MOVING && !g_chassis_front_pair.paused);
    CHECK(ChassisApp_ResumeFrontPair() == 0);
    position(0u,4.0f,0); refresh(); update(0.001f);
    CHECK(g_chassis_front_pair.paused != 0u);
    CHECK(ChassisApp_ResumeFrontPair() == 1);
    CHECK(g_chassis_front_pair.move_sequence == sequence);
    CHECK(g_chassis_front_pair.resume_ms != UINT32_MAX && !g_chassis_front_pair.paused);
    CHECK(ChassisApp_ResumeFrontPair() == 0);
    for (n = 0u; n < 2000u && g_chassis_front_state != CHASSIS_FRONT_DONE; n++) {
        float reference = g_chassis_front_pair.reference_turns;
        tick++;
        for (i = 0u; i < 2u; i++) { position(i, i == 0u ? reference : -reference, 0); }
        refresh(); update(0.001f);
    }
    CHECK(g_chassis_front_state == CHASSIS_FRONT_DONE);
    CHECK(g_chassis_front_pair.move_sequence == sequence);
    CLOSE(g_chassis_front_pair.actual_turns[0],10.0f,0.101f);
    CLOSE(g_chassis_front_pair.actual_turns[1],-10.0f,0.101f);
    /* Paused drives keep feedback-loss handling, and cannot resume after fault. */
    restart(1);
    CHECK(ChassisApp_MoveFrontPairTurnsWithPause(-10.0f,10.0f,0.4f) == CHASSIS_FRONT_START_OK);
    for (n = 0u; n < 1600u; n++) {
        float reference = g_chassis_front_pair.reference_turns;
        tick++;
        position(0u,-reference,0); position(1u,reference,0);
        refresh(); update(0.001f);
    }
    CHECK(g_chassis_front_pair.paused != 0u);
    tick += 51u; update(0.001f);
    CHECK(g_chassis_front_state == CHASSIS_FRONT_FAULT);
    CHECK(drive[0]->cmd == 0 && drive[1]->cmd == 0 && !g_chassis_front_pair.paused);
    CHECK(ChassisApp_ResumeFrontPair() == 0);
}
int main(void)
{
    DjiMotor_Init();
    CHECK(g_djimotor_reg_fail == 0u);
    CHECK(DJI_MODEL_3508.gear_ratio > 19.0f && DJI_MODEL_3508.gear_ratio < 20.0f);
    CHECK(CHASSIS_SWERVE_WHEEL_RADIUS_M == 0.0f); /* No geometry configured/required. */
    test_ready_atomic(); test_no_drive_control_before_request();
    test_signs_shared_reference(); test_sync_and_steer();
    test_winners_and_hold(); test_faults_and_bounds(); test_no_reference_or_time_completion();
    test_rear_ownership_and_done_dropout(); test_numeric_preflight();
    test_pause_at_fraction();
    printf("Front drive pair: %u checks, %u failures\n",checks,failures);
    return failures == 0u ? 0 : 1;
}
