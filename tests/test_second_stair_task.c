/* Exercise the real task through both stairs with asynchronous motion mocks. */
#include "My_task.h"
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
        CHECK(turns[0] == 35.0f && turns[1] == 0.0f && turns[2] == -35.0f);
    }
    sync_count++;
    sync_tick = tick;
    for (i = 0u; i < 3u; i++) {
        axes[i].state = AXIS_MOVING;
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
        CHECK(fabsf(turns - (pair_count == 2u ? 50.19518086f : 42.0f)) < 1e-4f);
        if (retry_pair != 0u && pair_count == 2u && tick < 100u) { return MOTORAPP_ERR_NOFB; }
    }
    if (pair_count == 1u || pair_count == 3u) {
        CHECK(pair.state == MOTORAPP_PAIR_DONE);
        CHECK(lift_count == (pair_count == 1u ? 1u : 2u));
        CHECK(axes[0].state == AXIS_MOVING); /* ID5 completion must not gate the extra leg */
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
    CHECK(a == &axes[0] && turns < 0.0f);
    CHECK(pair_count == 1u || pair_count == 3u);
    if (pair_count == 3u) {
        CHECK(turns == -35.0f && lift_count == 1u);
        lift_attempts++;
        if (retry_lift != 0u && tick < pair_tick + 4u) { return MOTORAPP_ERR_BUSY; }
    }
    if (pair.state != MOTORAPP_PAIR_DONE) {
        CHECK(pair.actual_turns[0] >= pair_target * 0.75f);
        CHECK(pair.actual_turns[1] >= pair_target * 0.75f);
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
            if (i == 0u && lift_count != 0u && lift_tick >= sync_tick) { continue; }
            axes[i].state = AXIS_IDLE;
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
    if (setjmp(finish) == 0) { upstairs_task(); }
    CHECK(updates == tick - 1u && sync_count == 3u);
    CHECK(g_upstairs_debug.first_stair_done == 1u && g_upstairs_debug.timing.status == 2u);
    CHECK(pair_count == (fault_pair != 0u || fault_lift != 0u ? 3u : 4u));
    CHECK(lift_count == 2u);
    CHECK(g_upstairs_debug.second_stair.id5_started == 1u);
    CHECK(g_upstairs_debug.second_stair.pair_timing[0].planner_ms == 1003u);
    CHECK(g_upstairs_debug.pair_timing[0].planner_ms == 1001u && g_upstairs_debug.pair_timing[1].planner_ms == 1002u);
    if (retry_lift != 0u) { CHECK(lift_attempts > 1u); }
    if (fault_pair != 0u || fault_extra != 0u || fault_lift != 0u) {
        CHECK(g_upstairs_debug.state == 3u && g_upstairs_debug.second_stair.status == 3u);
        CHECK(g_upstairs_debug.second_stair.total_ms == finished_at + offset - g_upstairs_debug.second_stair.step1_start_ms);
    } else if (wait_extra != 0u) {
        CHECK(g_upstairs_debug.state == 1u && g_upstairs_debug.step == 3u);
        CHECK(g_upstairs_debug.second_stair.status == 1u);
    } else {
        CHECK(g_upstairs_debug.state == 2u && g_upstairs_debug.second_stair.status == 2u);
        CHECK(g_upstairs_debug.second_stair.total_ms == g_upstairs_debug.second_stair.step1_ms +
              g_upstairs_debug.second_stair.step2_ms + g_upstairs_debug.second_stair.step3_ms);
        CHECK(g_upstairs_debug.second_stair.total_ms == finished_at + offset - g_upstairs_debug.second_stair.step1_start_ms);
        CHECK(g_upstairs_debug.second_stair.pair_timing[1].planner_ms == 1004u);
    }
    printf("Second stair task %s: %u failures\n", argv[1], failures);
    return failures == 0u ? 0 : 1;
}
