/* Run the real first-stair task through the pair/ID5 completion handoff. */
#include "My_task.h"
#include "ChassisTask.h"
#include <math.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static Axis_t axes[5];
static DjiMotor_t motors[5];
static const DjiMotor_Model_t model = {.gear_ratio = 36.0f};
static MotorApp_2006PairStatus_t pair;
static uint32_t tick, offset, pair_tick, lift_tick;
static unsigned int failures, sync_count, pair_count, lift_count, lift_attempts;
static unsigned int pair_late, lift_stalled, lift_fault, fallback, retry;
static float target;
static jmp_buf finish;
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); failures++; } } while (0)
uint32_t HAL_GetTick(void) { return tick + offset; }
uint32_t osKernelGetTickCount(void) { return tick; }
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
Axis_t *MotorApp_GetAxis(uint8_t i) { return i < 5u ? &axes[i] : NULL; }
const MotorApp_2006PairStatus_t *MotorApp_Get2006PairStatus(void) { return &pair; }
void MotorApp_Init(void)
{
    static const uint8_t ids[5] = {5u, 2u, 1u, 4u, 3u};
    unsigned int i;
    for (i = 0u; i < 5u; i++) {
        axes[i].motor = &motors[i];
        motors[i].id = ids[i];
        motors[i].model = &model;
    }
}
void MotorApp_Update(float dt) { CHECK(dt > 0.0009f && dt < 0.0011f); }
int MotorApp_Move3508TurnsSync(const float turns[3], float rate, float accel)
{
    CHECK(turns != NULL && rate > 0.0f && accel > 0.0f);
    sync_count++;
    CHECK(sync_count <= 2u);
    return MOTORAPP_OK;
}
int MotorApp_Move2006Pair(float turns)
{
    CHECK(sync_count == 2u && turns > 0.0f);
    if (pair_count != 0u) {
        CHECK(pair_count == 1u && pair.state == MOTORAPP_PAIR_DONE);
        CHECK(lift_count == 1u && axes[0].state == AXIS_IDLE);
        CHECK(axes[0].traj.active == 0u && tick >= lift_tick + 12u);
    }
    pair_count++;
    pair_tick = tick;
    target = turns;
    memset(&pair, 0, sizeof(pair));
    pair.state = MOTORAPP_PAIR_MOVING;
    pair.timing.start_ms = HAL_GetTick();
    pair.timing.status = 1u;
    return MOTORAPP_OK;
}
int MotorApp_MoveTurns(Axis_t *axis, float turns)
{
    CHECK(axis == &axes[0] && axis->motor->id == 5u && turns < 0.0f);
    CHECK(lift_count == 0u && pair_count == 1u);
    lift_attempts++;
    if (retry != 0u && lift_attempts < 3u) { return MOTORAPP_ERR_BUSY; }
    lift_count++;
    lift_tick = tick;
    axis->state = AXIS_MOVING;
    axis->traj.active = 1u;
    axis->traj.target = axis->motor->angle_rad + turns * 6.28318530718f / model.gear_ratio;
    return MOTORAPP_OK;
}
void osDelay(uint32_t ms)
{
    CHECK(ms == 1u);
    tick++;
    g_upstairs_start = 1u;
    if (pair_count == 1u) {
        uint32_t age = tick - pair_tick;
        pair.actual_turns[0] = target * 0.8f;
        pair.actual_turns[1] = fallback != 0u ? 0.0f : target * 0.8f;
        if (age >= (fallback != 0u ? 1u : (pair_late != 0u ? 25u : 5u))) {
            pair.state = MOTORAPP_PAIR_DONE;
            pair.timing.status = 2u;
        } else { pair.timing.total_ms = age; }
    }
    if (lift_count != 0u) {
        uint32_t age = tick - lift_tick;
        /* Reaching the position alone does not imply controller completion. */
        if (age >= 6u) { motors[0].angle_rad = axes[0].traj.target; }
        if (age >= 12u && lift_stalled == 0u) {
            axes[0].state = AXIS_IDLE;
            axes[0].traj.active = 0u;
        }
        if (lift_fault != 0u && age >= 4u) { axes[0].state = AXIS_FAULT; }
        if (pair_count == 1u && (axes[0].state == AXIS_MOVING || pair.state == MOTORAPP_PAIR_MOVING)) {
            CHECK(g_upstairs_debug.state == 0u && g_upstairs_debug.step == 3u);
        }
    }
    if (pair_count == 2u || tick >= 70u || g_upstairs_debug.state == 3u) { longjmp(finish, 1); }
}
int main(int argc, char **argv)
{
    (void)argc;
    pair_late = strcmp(argv[1], "pair_late") == 0 ? 1u : 0u;
    lift_stalled = strcmp(argv[1], "lift_stalled") == 0 ? 1u : 0u;
    lift_fault = strcmp(argv[1], "lift_fault") == 0 ? 1u : 0u;
    fallback = strcmp(argv[1], "fallback") == 0 ? 1u : 0u;
    retry = strcmp(argv[1], "retry") == 0 ? 1u : 0u;
    offset = strcmp(argv[1], "wrap") == 0 ? UINT32_MAX - 10u : 0u;
    if (setjmp(finish) == 0) { upstairs_task(); }
    CHECK(sync_count == 2u && lift_count == 1u);
    CHECK(lift_attempts == (retry != 0u ? 3u : 1u));
    CHECK(g_upstairs_debug.move_id5_result == MOTORAPP_OK);
    if (lift_fault != 0u) {
        CHECK(pair_count == 1u && g_upstairs_debug.state == 3u);
    } else if (lift_stalled != 0u) {
        CHECK(pair_count == 1u && g_upstairs_debug.step == 3u);
        CHECK(g_upstairs_debug.first_stair_done == 0u);
        CHECK(g_upstairs_debug.timing.step3_after_pair_ms > 0u);
    } else {
        CHECK(pair_count == 2u && g_upstairs_debug.step == 4u);
        CHECK(g_upstairs_debug.timing.step3_ms ==
            g_upstairs_debug.timing.step4_start_ms - g_upstairs_debug.timing.step2_end_ms);
    }
    printf("First stair handoff %s: %u failures\n", argv[1], failures);
    return failures == 0u ? 0 : 1;
}
