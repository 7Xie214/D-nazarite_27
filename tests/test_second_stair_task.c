/* Exercise the real task through both stairs with asynchronous motion mocks. */
#include "My_task.h"
#include "ChassisTask.h"
osStatus_t ChassisTask_RequestFrontMove(void) { return osErrorResource; }
osStatus_t ChassisTask_RequestFrontMoveWithPause(float l, float r, float p)
{ (void)l; (void)r; (void)p; return osErrorResource; }
osStatus_t ChassisTask_ResumeFrontMove(uint32_t sequence)
{ (void)sequence; return osErrorResource; }
uint32_t ChassisTask_GetFrontProgress(float actual[2])
{ actual[0] = 0.0f; actual[1] = 0.0f; return 0u; }
osStatus_t ChassisTask_RequestFrontMoveTurns(float left, float right)
{ (void)left; (void)right; return osErrorResource; }
uint32_t ChassisTask_PollReply(void) { return 0u; }
#include "MotorApp.h"
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static Axis_t axes[5];
static DjiMotor_t motors[5];
static const DjiMotor_Model_t model = {.gear_ratio = 3591.0f / 187.0f};
static const DjiMotor_Model_t model_2006 = {.gear_ratio = 36.0f};
static MotorApp_2006PairStatus_t pair;
static uint32_t tick, offset, sync_tick, pair_tick, lift_tick;
static unsigned int failures, updates, sync_count, pair_count, lift_count, lift_attempts;
static unsigned int retry_pair, fallback, retry_lift, fault_pair, fault_extra, fault_lift, wait_extra;
static unsigned int wait_step1, pair_pending;
static unsigned int id2_count, id2_attempts, retry_id2, wait_id2, fault_id2, reject_id2;
static uint32_t id2_tick;
static unsigned int id1_count, id1_attempts, retry_id1, wait_id1, fault_id1, reject_id1;
static uint32_t id1_tick;
static unsigned int stale_id2, fast_id2;
static float pair_target;
static uint32_t finished_at;
static jmp_buf finish;
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); failures++; } } while (0)
uint32_t HAL_GetTick(void) { return tick + offset; }
uint32_t osKernelGetTickCount(void) { return tick; }
Axis_t *MotorApp_GetAxis(uint8_t i) { return i < 5u ? &axes[i] : NULL; }
const MotorApp_2006PairStatus_t *MotorApp_Get2006PairStatus(void) { return &pair; }
void MotorApp_Init(void)
{
    static const uint8_t ids[5] = {5u, 2u, 1u, 4u, 3u};
    unsigned int i;
    for (i = 0u; i < 5u; i++) {
        axes[i].motor = &motors[i];
        motors[i].id = ids[i];
        motors[i].model = i < 3u ? &model : &model_2006;
    }
}
void MotorApp_Update(float dt) { CHECK(dt > 0.0009f && dt < 0.0011f); updates++; }
int MotorApp_Move3508TurnsSync(const float turns[3], float rate, float accel)
{
    unsigned int i;
    CHECK(rate > 0.0f && accel > 0.0f && sync_count < 3u);
    for (i = 0u; i < 3u; i++) {
        if (axes[i].state == AXIS_MOVING) { return MOTORAPP_ERR_BUSY; }
    }
    if (sync_count == 2u) {
        CHECK(pair_count == 2u && g_upstairs_debug.first_stair_done == 1u);
        CHECK(turns[0] == 36.5f && turns[1] == 0.0f && turns[2] == -36.5f);
    }
    sync_count++;
    sync_tick = tick;
    for (i = 0u; i < 3u; i++) {
        axes[i].state = AXIS_MOVING;
        axes[i].traj.active = 1u;
        axes[i].traj.target = motors[i].angle_rad + turns[i] * 6.28318530718f / model.gear_ratio;
    }
    return MOTORAPP_OK;
}
int MotorApp_Move2006Pair(float turns)
{
    unsigned int i;
    CHECK(pair_count < 4u);
    if (pair_count >= 2u) {
        CHECK(sync_count == 3u && tick > sync_tick + 6u);
        CHECK(fabsf(turns - (pair_count == 2u ? 50.19518086f : 40.0f)) < 1e-4f);
        if (pair_pending != 0u && pair_count == 2u) { return MOTORAPP_ERR_NOFB; }
        if (retry_pair != 0u && pair_count == 2u && tick < 100u) { return MOTORAPP_ERR_NOFB; }
    }
    if (pair_count == 1u || pair_count == 3u) {
        CHECK(pair.state == MOTORAPP_PAIR_DONE);
        CHECK(lift_count == (pair_count == 1u ? 1u : 2u));
        CHECK(axes[0].state == (pair_count == 1u ? AXIS_IDLE : AXIS_MOVING));
    }
    pair_count++;
    pair_tick = tick;
    pair_target = turns;
    memset(&pair, 0, sizeof(pair));
    pair.state = MOTORAPP_PAIR_MOVING;
    pair.timing.start_ms = HAL_GetTick();
    pair.timing.status = 1u;
    pair.timing.planner_ms = 1000u + pair_count;
    for (i = 3u; i < 5u; i++) { axes[i].state = AXIS_MOVING; }
    return MOTORAPP_OK;
}
int MotorApp_MoveTurns(Axis_t *a, float turns)
{
    if (a == &axes[1]) {
        CHECK(a->motor->id == 2u && fabsf(turns - TEST_SECOND_ID2_ROTOR_TURNS) < 1e-4f);
        CHECK(pair_count == 4u && pair.state == MOTORAPP_PAIR_DONE);
        CHECK(g_upstairs_debug.state == 1u && g_upstairs_debug.step == 4u);
        CHECK(id2_count == 0u);
        id2_attempts++;
        if (reject_id2 != 0u) { return MOTORAPP_ERR_FAULT; }
        if (retry_id2 != 0u && tick < pair_tick + 9u) { return MOTORAPP_ERR_BUSY; }
        id2_count++;
        id2_tick = tick;
        a->state = AXIS_MOVING;
        a->traj.active = 1u;
        a->traj.target = a->motor->angle_rad + turns * 6.28318530718f / model.gear_ratio;
        return MOTORAPP_OK;
    }
    if (a == &axes[2]) {
        CHECK(a->motor->id == 1u && turns == 36.5f);
        CHECK(id2_count == 1u && axes[1].state == AXIS_IDLE && tick >= id2_tick + 8u);
        if (stale_id2 != 0u || fast_id2 != 0u) { CHECK(tick >= id2_tick + 15u); }
        CHECK(g_upstairs_debug.state == 1u && g_upstairs_debug.step == 5u);
        CHECK(id1_count == 0u);
        id1_attempts++;
        if (reject_id1 != 0u) { return MOTORAPP_ERR_FAULT; }
        if (retry_id1 != 0u && tick < id2_tick + 12u) { return MOTORAPP_ERR_BUSY; }
        id1_count++;
        id1_tick = tick;
        a->state = AXIS_MOVING;
        a->traj.active = 1u;
        a->traj.target = a->motor->angle_rad + turns * 6.28318530718f / model.gear_ratio;
        return MOTORAPP_OK;
    }
    CHECK(a == &axes[0] && turns < 0.0f);
    CHECK(pair_count == 1u || pair_count == 3u);
    if (pair_count == 3u) {
        CHECK(turns == -36.5f && lift_count == 1u);
        lift_attempts++;
        if (retry_lift != 0u && tick < pair_tick + 4u) { return MOTORAPP_ERR_BUSY; }
    }
    if (pair.state != MOTORAPP_PAIR_DONE) {
        float threshold = pair_target * (pair_count == 1u ? 0.60f : 0.75f);
        CHECK(pair.actual_turns[0] >= threshold);
        CHECK(pair.actual_turns[1] >= threshold);
    }
    lift_count++;
    lift_tick = tick;
    a->state = AXIS_MOVING;
    a->traj.target = a->motor->angle_rad + turns * 6.28318530718f / model.gear_ratio;
    return MOTORAPP_OK;
}
void osDelay(uint32_t ms)
{
    unsigned int i;
    CHECK(ms == 1u);
    tick++;
    g_upstairs_start = 1u;
    if (sync_count != 0u && tick >= sync_tick + 6u) {
        for (i = 0u; i < 3u; i++) {
            if (i == 1u && id2_count != 0u) { continue; }
            if (i == 2u && id1_count != 0u) { continue; }
            if (i == 0u && lift_count != 0u && lift_tick >= sync_tick) { continue; }
            if (sync_count == 3u && i == 2u && wait_step1 != 0u) {
                axes[i].state = AXIS_MOVING;
                axes[i].traj.active = wait_step1 == 1u ? 1u : 0u;
                motors[i].angle_rad = axes[i].traj.target - (wait_step1 == 2u ?
                    0.2f * 6.28318530718f / model.gear_ratio : 0.0f);
                motors[i].raw_speed = wait_step1 == 3u ? 50 : 0;
                motors[i].speed_rad_s = (float)motors[i].raw_speed * 6.28318530718f / (60.0f * model.gear_ratio);
                continue;
            }
            axes[i].state = AXIS_IDLE;
            axes[i].traj.active = 0u;
            motors[i].angle_rad = axes[i].traj.target;
        }
    }
    if (lift_count != 0u && tick >= lift_tick + 12u && lift_tick >= sync_tick) {
        axes[0].state = AXIS_IDLE;
        motors[0].angle_rad = axes[0].traj.target;
    }
    if (pair_count != 0u) {
        uint32_t age = tick - pair_tick;
        pair.actual_turns[0] = pair_target * (age >= 5u ? 1.0f : 0.8f);
        pair.actual_turns[1] = pair_target * (age >= 5u ? 0.9f : (age >= 2u ? 0.8f : 0.74f));
        if (age == 2u) { pair.actual_turns[0] = pair_target * 0.75f; pair.actual_turns[1] = pair_target * 0.75f; }
        if (fallback != 0u && pair_count == 3u && age == 1u) {
            pair.actual_turns[0] = pair_target; pair.actual_turns[1] = pair_target * 0.2f;
            pair.state = MOTORAPP_PAIR_DONE;
        }
        if (age >= 5u && !(wait_extra != 0u && pair_count == 4u)) { pair.state = MOTORAPP_PAIR_DONE; }
        if ((fault_pair != 0u && pair_count == 3u && age >= 3u) ||
            (fault_extra != 0u && pair_count == 4u && age >= 2u)) {
            pair.state = MOTORAPP_PAIR_FAULT; pair.fault = MOTORAPP_PAIR_FAULT_OFFLINE;
        }
        if (pair.timing.status == 1u) {
            pair.timing.total_ms = age;
            if (pair.state == MOTORAPP_PAIR_DONE) { pair.timing.status = 2u; }
            if (pair.state == MOTORAPP_PAIR_FAULT) { pair.timing.status = 3u; }
        }
        if (pair.state == MOTORAPP_PAIR_DONE) { axes[3].state = AXIS_IDLE; axes[4].state = AXIS_IDLE; }
    }
    if (fault_lift != 0u && lift_count == 2u && tick >= lift_tick + 2u) { axes[0].state = AXIS_FAULT; }
    if (id2_count != 0u) {
        if (tick >= id2_tick + 8u && wait_id2 == 0u) {
            axes[1].state = AXIS_IDLE;
            axes[1].traj.active = 0u;
            motors[1].angle_rad = axes[1].traj.target;
            if (stale_id2 != 0u && tick < id2_tick + 15u) {
                motors[1].angle_rad += 0.5f * 6.28318530718f / model.gear_ratio;
            }
            motors[1].raw_speed = fast_id2 != 0u && tick < id2_tick + 15u ? 30 : 0;
            motors[1].speed_rad_s = (float)motors[1].raw_speed * 6.28318530718f / (60.0f * model.gear_ratio);
        }
        if (fault_id2 != 0u && tick >= id2_tick + 2u) { axes[1].state = AXIS_FAULT; }
        if (tick < id2_tick + 8u || wait_id2 != 0u) {
            CHECK(g_upstairs_debug.second_stair.status == 1u || g_upstairs_debug.state == 3u);
        }
    }
    if (id1_count != 0u) {
        if (tick >= id1_tick + 8u && wait_id1 == 0u) {
            axes[2].state = AXIS_IDLE;
            axes[2].traj.active = 0u;
            motors[2].angle_rad = axes[2].traj.target;
        }
        if (fault_id1 != 0u && tick >= id1_tick + 2u) { axes[2].state = AXIS_FAULT; }
        if (tick < id1_tick + 8u || wait_id1 != 0u) {
            CHECK(g_upstairs_debug.second_stair.status == 1u || g_upstairs_debug.state == 3u);
        }
    }
    if (finished_at == 0u && g_upstairs_debug.second_stair.status >= 2u) { finished_at = tick - 1u; }
    if (tick >= 220u) { longjmp(finish, 1); }
}
int main(int argc, char **argv)
{
    (void)argc;
    offset = strcmp(argv[1], "wrap") == 0 ? UINT32_MAX - 40u : 0u;
    retry_pair = strcmp(argv[1], "pair_retry") == 0 ? 1u : 0u;
    fallback = (strcmp(argv[1], "fallback") == 0 || strcmp(argv[1], "fallback_retry") == 0) ? 1u : 0u;
    retry_lift = strcmp(argv[1], "fallback_retry") == 0 ? 1u : 0u;
    fault_pair = strcmp(argv[1], "pair_fault") == 0 ? 1u : 0u;
    fault_extra = strcmp(argv[1], "extra_fault") == 0 ? 1u : 0u;
    fault_lift = strcmp(argv[1], "id5_fault") == 0 ? 1u : 0u;
    wait_extra = strcmp(argv[1], "wait_extra") == 0 ? 1u : 0u;
    wait_step1 = strcmp(argv[1], "wait_planner") == 0 ? 1u :
        (strcmp(argv[1], "wait_position") == 0 ? 2u : (strcmp(argv[1], "wait_speed") == 0 ? 3u : 0u));
    pair_pending = strcmp(argv[1], "pair_pending") == 0 ? 1u : 0u;
    retry_id2 = strcmp(argv[1], "id2_retry") == 0 ? 1u : 0u;
    wait_id2 = strcmp(argv[1], "wait_id2") == 0 ? 1u : 0u;
    fault_id2 = strcmp(argv[1], "id2_fault") == 0 ? 1u : 0u;
    reject_id2 = strcmp(argv[1], "id2_rejected") == 0 ? 1u : 0u;
    retry_id1 = strcmp(argv[1], "id1_retry") == 0 ? 1u : 0u;
    wait_id1 = strcmp(argv[1], "wait_id1") == 0 ? 1u : 0u;
    fault_id1 = strcmp(argv[1], "id1_fault") == 0 ? 1u : 0u;
    reject_id1 = strcmp(argv[1], "id1_rejected") == 0 ? 1u : 0u;
    stale_id2 = strcmp(argv[1], "id2_stale") == 0 ? 1u : 0u;
    fast_id2 = strcmp(argv[1], "id2_fast_idle") == 0 ? 1u : 0u;
    if (setjmp(finish) == 0) { upstairs_task(); }
    CHECK(updates == tick - 1u && sync_count == 3u);
    CHECK(g_upstairs_debug.first_stair_done == 1u && g_upstairs_debug.timing.status == 2u);
    if (wait_step1 != 0u || pair_pending != 0u) {
        CHECK(g_upstairs_debug.state == 1u && g_upstairs_debug.second_stair.status == 1u);
        CHECK(pair_count == 2u && lift_count == 1u);
        CHECK(g_upstairs_debug.second_stair.step1_axis[2].motor_id == 1u);
        CHECK(g_upstairs_debug.second_stair.step1_axis[0].done_ms > 0u);
        if (wait_step1 != 0u) {
            CHECK(g_upstairs_debug.step == 1u && g_upstairs_debug.issued == 1u);
            CHECK(g_upstairs_debug.second_stair.pair_request_count[0] == 0u);
            CHECK(g_upstairs_debug.second_stair.pair_move_result[0] == 1);
            CHECK(g_upstairs_debug.second_stair.step1_axis[2].done_ms == 0u);
            CHECK(g_upstairs_debug.second_stair.step1_axis[2].last_reason == wait_step1);
            if (wait_step1 == 2u) { CHECK(g_upstairs_debug.second_stair.step1_axis[2].position_ms > 0u); }
            if (wait_step1 == 3u) { CHECK(g_upstairs_debug.second_stair.step1_axis[2].speed_ms > 0u); }
        } else {
            CHECK(g_upstairs_debug.step == 2u && g_upstairs_debug.issued == 0u);
            CHECK(g_upstairs_debug.second_stair.step1_axis[2].done_ms > 0u);
            CHECK(g_upstairs_debug.second_stair.pair_request_count[0] > 1u);
            CHECK(g_upstairs_debug.second_stair.pair_move_result[0] == MOTORAPP_ERR_NOFB);
        }
        printf("Second stair task %s: %u failures\n", argv[1], failures);
        return failures == 0u ? 0 : 1;
    }
    CHECK(pair_count == (fault_pair != 0u || fault_lift != 0u ? 3u : 4u));
    CHECK(lift_count == 2u);
    CHECK(g_upstairs_debug.second_stair.id5_started == 1u);
    CHECK(g_upstairs_debug.second_stair.step1_axis[2].done_ms > 0u);
    CHECK(g_upstairs_debug.second_stair.pair_request_count[0] > 0u);
    CHECK(g_upstairs_debug.second_stair.pair_move_result[0] == MOTORAPP_OK);
    if (pair_count == 4u) { CHECK(g_upstairs_debug.second_stair.pair_move_result[1] == MOTORAPP_OK); }
    CHECK(g_upstairs_debug.second_stair.pair_timing[0].planner_ms == 1003u);
    CHECK(g_upstairs_debug.pair_timing[0].planner_ms == 1001u && g_upstairs_debug.pair_timing[1].planner_ms == 1002u);
    if (retry_lift != 0u) { CHECK(lift_attempts > 1u); }
    if (fault_pair != 0u || fault_extra != 0u || fault_lift != 0u || wait_extra != 0u) {
        CHECK(id2_attempts == 0u && id2_count == 0u);
    } else if (reject_id2 != 0u) {
        CHECK(id2_attempts == 1u && id2_count == 0u);
        CHECK(g_upstairs_debug.second_stair.id2_move_result == MOTORAPP_ERR_FAULT);
    } else {
        CHECK(id2_count == 1u);
        CHECK(id2_attempts == (retry_id2 != 0u ? 4u : 1u));
        CHECK(g_upstairs_debug.second_stair.id2_move_result == MOTORAPP_OK);
    }
    if (fault_pair != 0u || fault_extra != 0u || fault_lift != 0u || wait_extra != 0u ||
        wait_id2 != 0u || fault_id2 != 0u || reject_id2 != 0u) {
        CHECK(id1_count == 0u && id1_attempts == 0u);
    } else if (reject_id1 != 0u) {
        CHECK(id1_count == 0u && id1_attempts == 1u);
        CHECK(g_upstairs_debug.second_stair.id1_move_result == MOTORAPP_ERR_FAULT);
    } else {
        CHECK(id1_count == 1u);
        CHECK(id1_attempts == (retry_id1 != 0u ? 4u : 1u));
        CHECK(g_upstairs_debug.second_stair.id1_move_result == MOTORAPP_OK);
    }
    if (fault_pair != 0u || fault_extra != 0u || fault_lift != 0u || fault_id2 != 0u || reject_id2 != 0u ||
        fault_id1 != 0u || reject_id1 != 0u) {
        CHECK(g_upstairs_debug.state == 3u && g_upstairs_debug.second_stair.status == 3u);
        CHECK(g_upstairs_debug.second_stair.total_ms == finished_at + offset - g_upstairs_debug.second_stair.step1_start_ms);
    } else if (wait_extra != 0u) {
        CHECK(g_upstairs_debug.state == 1u && g_upstairs_debug.step == 3u);
        CHECK(g_upstairs_debug.second_stair.status == 1u);
    } else if (wait_id2 != 0u) {
        CHECK(g_upstairs_debug.state == 1u && g_upstairs_debug.step == 4u);
        CHECK(g_upstairs_debug.second_stair.status == 1u && axes[1].state == AXIS_MOVING);
        CHECK(g_upstairs_debug.second_stair.step4_ms > 0u);
    } else if (wait_id1 != 0u) {
        CHECK(g_upstairs_debug.state == 1u && g_upstairs_debug.step == 5u);
        CHECK(g_upstairs_debug.second_stair.status == 1u && axes[2].state == AXIS_MOVING);
        CHECK(g_upstairs_debug.second_stair.step5_ms > 0u);
    } else {
        CHECK(g_upstairs_debug.state == 2u && g_upstairs_debug.second_stair.status == 2u);
        CHECK(g_upstairs_debug.second_stair.total_ms == g_upstairs_debug.second_stair.step1_ms +
              g_upstairs_debug.second_stair.step2_ms + g_upstairs_debug.second_stair.step3_ms +
              g_upstairs_debug.second_stair.step4_ms + g_upstairs_debug.second_stair.step5_ms);
        CHECK(g_upstairs_debug.second_stair.step4_ms ==
            (stale_id2 != 0u || fast_id2 != 0u ? 16u : (retry_id2 != 0u ? 12u : 9u)));
        CHECK(g_upstairs_debug.second_stair.id2_handoff_done == 1u);
        CHECK(fabsf(g_upstairs_debug.second_stair.id2_actual_turns - TEST_SECOND_ID2_ROTOR_TURNS) < 1e-4f);
        CHECK(fabsf(g_upstairs_debug.second_stair.id2_error_turns) < 1e-4f);
        CHECK(g_upstairs_debug.second_stair.id2_speed_rpm == 0);
        CHECK(g_upstairs_debug.second_stair.step5_ms == (retry_id1 != 0u ? 12u : 9u));
        CHECK(g_upstairs_debug.second_stair.total_ms == finished_at + offset - g_upstairs_debug.second_stair.step1_start_ms);
        CHECK(g_upstairs_debug.second_stair.pair_timing[1].planner_ms == 1004u);
    }
    printf("Second stair task %s: %u failures\n", argv[1], failures);
    return failures == 0u ? 0 : 1;
}
