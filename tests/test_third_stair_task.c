/* Real upstairs state machine: all prior moves finish, then notify chassis. */
#include "My_task.h"
#include "ChassisTask.h"
#include "PID_Config.h"
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

static Axis_t axes[5];
static DjiMotor_t motors[5];
static const DjiMotor_Model_t model = {.gear_ratio = 36.0f};
static MotorApp_2006PairStatus_t pair;
static uint32_t tick;
static unsigned int failures, requests, polls, accepted, syncs, pairs;
static unsigned int lift_attempts, pair_attempts;
static uint32_t lift_tick, pair_tick;
static int lift_retry, lift_rejected, wait_id5, wait_id1, lift_fault, stale_id5;
static int pair_retry, pair_fault, wait_pair;
static unsigned int final_attempts, final_count;
static uint32_t final_tick;
static int final_retry, final_rejected, final_fault, wait_final;
static int wait_final_id5, final_fault_id5, stale_final_id5;
static unsigned int fourth_requests, fourth_accepted, fourth_polls;
static int fourth_retry, fourth_reject, fourth_error, wait_fourth;
static unsigned int fourth_lift_attempts, fourth_pair_attempts;
static uint32_t fourth_lift_tick, fourth_pair_tick;
static int fourth_lift_retry, fourth_lift_reject, fourth_wait_id5, fourth_wait_id1;
static int fourth_stale_id5, fourth_lift_fault, fourth_pair_retry, fourth_pair_reject;
static int fourth_pair_fault, fourth_wait_pair;
static unsigned int end_id1_attempts, end_id1_accepted, end_front_requests, end_front_accepted, end_front_polls;
static uint32_t end_id1_tick;
static int end_id1_retry, end_id1_reject, end_id1_wait, end_id1_stale, end_id1_fast, end_id1_active, end_id1_fault;
static int end_front_retry, end_front_reject, end_front_wait, end_front_error;
static int end_id5_wait, end_id5_stale, end_id5_fault;
static unsigned int link_id2_attempts, link_id2_accepted;
static uint32_t link_id2_tick;
static uint32_t mock_sequence = 2u, end_front_tick;
static float mock_actual[2] = {15.2453059f, -15.2453059f};
static float mock_front_target[2];
static int link_crossed, link_early_done, link_front_wait_after_trigger, mock_replied;
static float mock_pause_fraction;
static int mock_resumed;
static uint32_t mock_resume_tick;
static unsigned int mock_resume_requests;
static unsigned int mock_resume_attempts;
static int resume_retry, link_id2_reset;
static float link_original_target;
static unsigned int return_attempts, return_accepted;
static uint32_t return_tick;
static float return_original_target;
static int return_retry, return_reject, return_wait, return_stale, return_fast, return_active, return_fault, return_reset;

static int link_id2_retry, link_id2_reject, link_id2_wait, link_id2_stale, link_id2_fast, link_id2_active, link_id2_fault;
static int retry, error_reply, reject;
static jmp_buf stop;
#define CHECK(x) do { if (!(x)) { failures++; printf("FAIL %d: %s\n", __LINE__, #x); } } while (0)
uint32_t HAL_GetTick(void) { return tick; }
uint32_t osKernelGetTickCount(void) { return tick; }
void osDelay(uint32_t ms)
{
    CHECK(ms == 1u); tick++;
    g_upstairs_start = 1u;
    if (syncs == 4u) {
        uint32_t age = tick - lift_tick;
        if (age >= 3u && !wait_id5) {
            axes[0].state = AXIS_IDLE; axes[0].traj.active = 0u;
            motors[0].angle_rad = axes[0].traj.target;
            if (stale_id5 && age < 16u) { motors[0].angle_rad -= 0.2f*6.28318530718f/36.0f; }
        }
        if (age >= 10u && !wait_id1 && final_count == 0u) {
            axes[2].state = AXIS_IDLE; axes[2].traj.active = 0u;
            motors[2].angle_rad = axes[2].traj.target;
        }
        if (lift_fault && age >= 4u) { axes[0].state = AXIS_FAULT; }
    }
    if (pairs == 5u) {
        uint32_t age = tick - pair_tick;
        if (age >= 8u && !wait_pair) { pair.state = MOTORAPP_PAIR_DONE; }
        if (pair_fault && age >= 3u) { pair.state = MOTORAPP_PAIR_FAULT; }
    }
    if (final_count != 0u && syncs == 5u) {
        uint32_t age = tick - final_tick;
        if (age >= 3u && !wait_final_id5) {
            axes[0].state = AXIS_IDLE; axes[0].traj.active = 0u;
            motors[0].angle_rad = axes[0].traj.target;
            if (stale_final_id5 && age < 14u) { motors[0].angle_rad += 0.2f*6.28318530718f/36.0f; }
        }
        if (age >= 8u && !wait_final) {
            axes[2].state = AXIS_IDLE; axes[2].traj.active = 0u;
            motors[2].angle_rad = axes[2].traj.target;
        }
        if (final_fault && age >= 3u) { axes[2].state = AXIS_FAULT; }
        if (final_fault_id5 && age >= 3u) { axes[0].state = AXIS_FAULT; }
    }
    if (syncs == 6u && end_id1_accepted == 0u) {
        uint32_t age = tick - fourth_lift_tick;
        if (age >= 4u && !fourth_wait_id5) {
            axes[0].state = AXIS_IDLE; axes[0].traj.active = 0u;
            motors[0].angle_rad = axes[0].traj.target;
            if (fourth_stale_id5 && age < 18u) { motors[0].angle_rad -= 0.2f*6.28318530718f/36.0f; }
        }
        if (age >= 12u && !fourth_wait_id1) {
            axes[2].state = AXIS_IDLE; axes[2].traj.active = 0u;
            motors[2].angle_rad = axes[2].traj.target;
        }
        if (fourth_lift_fault && age >= 5u) { axes[0].state = AXIS_FAULT; }
    }
    if (pairs == 6u) {
        uint32_t age = tick - fourth_pair_tick;
        if (age >= 7u && !fourth_wait_pair) { pair.state = MOTORAPP_PAIR_DONE; }
        if (fourth_pair_fault && age >= 3u) { pair.state = MOTORAPP_PAIR_FAULT; }
    }
    if (end_id1_accepted != 0u && return_accepted == 0u) {
        uint32_t age = tick - end_id1_tick;
        if (age >= 12u && !end_id5_wait) {
            axes[0].state = AXIS_IDLE; axes[0].traj.active = 0u;
            motors[0].angle_rad = axes[0].traj.target;
            if (end_id5_stale && age < 24u) { motors[0].angle_rad += 0.2f*6.28318530718f/36.0f; }
        }
        if (end_id5_fault && age >= 4u) { axes[0].state = AXIS_FAULT; }
        if (age >= 8u && !end_id1_wait) {
            axes[2].state = AXIS_IDLE; axes[2].traj.active = 0u;
            motors[2].angle_rad = axes[2].traj.target;
            if (end_id1_stale && age < 16u) { motors[2].angle_rad -= 0.2f*6.28318530718f/36.0f; }
            if (end_id1_active && age < 18u) { axes[2].traj.active = 1u; }
            motors[2].speed_rad_s = end_id1_fast && age < 20u ? 1.0f : 0.0f;
        }
        if (end_id1_fault && age >= 4u) { axes[2].state = AXIS_FAULT; }
    }
    if (return_accepted != 0u) {
        uint32_t age = tick - return_tick;
        if (age >= 8u && !return_wait) {
            axes[2].state = AXIS_IDLE; axes[2].traj.active = 0u;
            motors[2].angle_rad = return_original_target;
            if (return_stale && age < 16u) { motors[2].angle_rad -= 0.2f*6.28318530718f/36.0f; }
            if (return_active && age < 18u) { axes[2].traj.active = 1u; }
            motors[2].speed_rad_s = return_fast && age < 20u ? 1.0f : 0.0f;
            if (return_reset) {
                motors[2].angle_rad = return_original_target - 35.0f*6.28318530718f/36.0f;
                axes[2].traj.target = motors[2].angle_rad;
            }
        }
        if (return_fault && age >= 4u) { axes[2].state = AXIS_FAULT; }
        if (!return_fault && (age < 8u || return_wait || return_reset || (return_stale && age < 16u) ||
            (return_active && age < 18u) || (return_fast && age < 20u))) {
            CHECK(g_upstairs_debug.fourth_stair.status == 1u);
        }
    }
    if (link_id2_accepted != 0u) {
        uint32_t age = tick - link_id2_tick;
        if (age < 8u) {
            CHECK(axes[1].traj.v_max == UPSTAIRS_ID2_LINK_V_MAX_RAD_S);
            CHECK(axes[1].traj.a_max == UPSTAIRS_ID2_LINK_A_MAX_RAD_S2);
        }
        if (age >= 8u && !link_id2_wait) {
            axes[1].state = AXIS_IDLE; axes[1].traj.active = 0u;
            motors[1].angle_rad = axes[1].traj.target;
            if (link_id2_stale && age < 16u) { motors[1].angle_rad -= 0.2f*6.28318530718f/36.0f; }
            if (link_id2_active && age < 18u) { axes[1].traj.active = 1u; }
            motors[1].speed_rad_s = link_id2_fast && age < 20u ? 1.0f : 0.0f;
            if (link_id2_reset) {
                motors[1].angle_rad = link_original_target - 3.84f*6.28318530718f/36.0f;
                axes[1].traj.target = motors[1].angle_rad;
            }
        }
        if (link_id2_fault && age >= 4u) { axes[1].state = AXIS_FAULT; }
    }
    if (end_front_accepted != 0u && (!end_front_wait || link_front_wait_after_trigger)) {
        uint32_t age = tick - end_front_tick;
        if (age >= 3u) {
            float q0 = 0.0f, q1 = 0.0f;
            mock_sequence = 3u;
            if (link_early_done && age >= 4u) { q0 = 1.0f; q1 = 0.2f; }
            else if (age == 4u) { q0 = 0.35f; q1 = 0.349f; }
            else if (age == 5u) { q0 = 0.349f; q1 = 0.35f; }
            else if (age == 6u) { q0 = 0.350001f; q1 = 0.350001f; link_crossed = 1; }
            else if (age == 7u) { q0 = 0.3f; q1 = 0.3f; }
            else if (age >= 8u) { q0 = 0.5f; q1 = 0.5f; }
            if (mock_pause_fraction > 0.0f && age >= 8u) {
                q0 = 0.4f; q1 = 0.4f;
                if (mock_resumed) {
                    uint32_t resumed_age = tick - mock_resume_tick;
                    if (resumed_age >= 6u && !link_front_wait_after_trigger) { q0 = 0.75f; q1 = 0.7f; }
                    if (resumed_age >= 10u && !link_front_wait_after_trigger) { q0 = 1.0f; q1 = 0.95f; }
                }
            } else {
                if (!link_early_done && !link_front_wait_after_trigger && age >= 16u) { q0 = 0.75f; q1 = 0.7f; }
                if (!link_early_done && !link_front_wait_after_trigger && age >= 20u) { q0 = 1.0f; q1 = 0.95f; }
            }
            mock_actual[0] = q0 * mock_front_target[0]; mock_actual[1] = q1 * mock_front_target[1];
        }
    }
    if (tick > 200u) { longjmp(stop, 1); }
}
Axis_t *MotorApp_GetAxis(uint8_t i) { return &axes[i]; }
const MotorApp_2006PairStatus_t *MotorApp_Get2006PairStatus(void) { return &pair; }
void MotorApp_Init(void)
{
    static const uint8_t ids[5] = {5u,2u,1u,4u,3u};
    uint8_t i;
    for (i=0u; i<5u; i++) { axes[i].traj.v_max=30.0f; axes[i].traj.a_max=30.0f; axes[i].motor=&motors[i]; motors[i].model=&model; motors[i].id=ids[i]; }
}
void MotorApp_Update(float dt) { CHECK(dt > 0.0f); }
int MotorApp_Move3508TurnsSync(const float turns[3], float rate, float accel)
{
    uint8_t i; (void)rate; (void)accel;
    if (syncs == 3u) {
        CHECK(accepted == 1u && polls == 4u && !error_reply);
        CHECK(turns[0] == 36.5f && turns[1] == 0.0f && turns[2] == -36.5f);
        lift_attempts++;
        if (lift_rejected) { return MOTORAPP_ERR_PARAM; }
        if (lift_retry && lift_attempts < 3u) { return MOTORAPP_ERR_BUSY; }
        lift_tick = tick;
    }
    if (syncs == 4u) {
        CHECK(pairs == 5u && pair.state == MOTORAPP_PAIR_DONE);
        CHECK(turns[0] == -36.5f && turns[1] == 0.0f && turns[2] == 36.5f);
        CHECK(tick - pair_tick >= 8u && final_count == 0u);
        final_attempts++;
        if (final_rejected) { return MOTORAPP_ERR_PARAM; }
        if (final_retry && final_attempts < 3u) { return MOTORAPP_ERR_BUSY; }
        final_count++;
        final_tick = tick;
    }
    if (syncs == 5u) {
        CHECK(fourth_accepted == 1u && fourth_polls == 4u && !fourth_error);
        CHECK(turns[0] == 36.5f && turns[1] == 0.0f && turns[2] == -36.5f);
        fourth_lift_attempts++;
        if (fourth_lift_reject) { return MOTORAPP_ERR_PARAM; }
        if (fourth_lift_retry && fourth_lift_attempts < 3u) { return MOTORAPP_ERR_BUSY; }
        fourth_lift_tick = tick;
    }
    if (syncs == 6u) {
        CHECK(pair.state == MOTORAPP_PAIR_DONE && pairs == 6u);
        CHECK(turns[0] == -36.5f && turns[1] == 0.0f && turns[2] == 36.5f);
        end_id1_attempts++;
        if (end_id1_reject) { return MOTORAPP_ERR_PARAM; }
        if (end_id1_retry && end_id1_attempts < 3u) { return MOTORAPP_ERR_BUSY; }
        end_id1_accepted++; end_id1_tick = tick;
    }
    syncs++;
    for (i=0u; i<3u; i++) {
        axes[i].traj.target += turns[i]*6.28318530718f/36.0f;
        if (syncs >= 4u) { axes[i].state = AXIS_MOVING; axes[i].traj.active = 1u; }
        else { motors[i].angle_rad=axes[i].traj.target; }
    }
    return MOTORAPP_OK;
}
int MotorApp_MoveTurns(Axis_t *a, float turns)
{
    if (end_front_accepted != 0u && a == &axes[2]) {
        CHECK(mock_replied && mock_resumed && tick - mock_resume_tick >= 10u);
        CHECK(g_upstairs_debug.fourth_stair.front_final_reply == CHASSIS_TASK_DONE_FLAG);
        CHECK(g_upstairs_debug.fourth_stair.status == 1u);
        CHECK(axes[1].state == AXIS_IDLE && axes[1].traj.active == 0u);
        CHECK(fabsf(motors[1].angle_rad - link_original_target) < 0.00001f);
        CHECK(a->motor->id == 1u && fabsf(turns - 70.930979f) < 0.00001f);
        return_attempts++;
        if (return_reject) { return MOTORAPP_ERR_PARAM; }
        if (return_retry && return_attempts < 3u) { return MOTORAPP_ERR_BUSY; }
        CHECK(return_accepted == 0u);
        return_accepted++; return_tick = tick;
        a->traj.target = a->motor->angle_rad + turns*6.28318530718f/36.0f;
        return_original_target = a->traj.target;
        a->state = AXIS_MOVING; a->traj.active = 1u;
        return MOTORAPP_OK;
    }
    if (end_front_accepted != 0u && a == &axes[1]) {
        CHECK(mock_sequence == 3u && (link_crossed || link_early_done));
        CHECK(a->motor->id == 2u && fabsf(turns - 31.6812f) < 0.00001f);
        link_id2_attempts++;
        if (link_id2_reject) { return MOTORAPP_ERR_PARAM; }
        if (link_id2_retry && link_id2_attempts < 3u) { return MOTORAPP_ERR_BUSY; }
        link_id2_accepted++; link_id2_tick = tick;
        a->traj.target = a->motor->angle_rad + turns*6.28318530718f/36.0f;
        link_original_target = a->traj.target;
        a->state = AXIS_MOVING; a->traj.active = 1u;
        return MOTORAPP_OK;
    }
    CHECK(pairs < 5u); /* The final two motors must use one atomic sync request. */
    a->traj.target = a->motor->angle_rad + turns*6.28318530718f/36.0f;
    a->motor->angle_rad = a->traj.target;
    return MOTORAPP_OK;
}
int MotorApp_Move2006Pair(float turns)
{
    if (pairs == 5u) {
        CHECK(syncs == 6u && axes[0].state == AXIS_IDLE && axes[2].state == AXIS_IDLE);
        CHECK(axes[0].traj.active == 0u && axes[2].traj.active == 0u);
        CHECK(fabsf(axes[0].traj.target - motors[0].angle_rad) <= 0.1f*6.28318530718f/36.0f);
        CHECK(fabsf(axes[2].traj.target - motors[2].angle_rad) <= 0.1f*6.28318530718f/36.0f);
        CHECK(fabsf(turns - 50.19518086f) < 0.0001f);
        CHECK(tick - fourth_lift_tick >= (fourth_stale_id5 ? 18u : 12u));
        fourth_pair_attempts++;
        if (fourth_pair_reject) { return MOTORAPP_ERR_PARAM; }
        if (fourth_pair_retry && fourth_pair_attempts < 3u) { return MOTORAPP_ERR_NOFB; }
        fourth_pair_tick = tick;
    }
    if (pairs == 4u) {
        CHECK(syncs == 4u && axes[0].state == AXIS_IDLE && axes[2].state == AXIS_IDLE);
        CHECK(fabsf(axes[0].traj.target - motors[0].angle_rad) <= 0.1f*6.28318530718f/36.0f);
        CHECK(fabsf(axes[2].traj.target - motors[2].angle_rad) <= 0.1f*6.28318530718f/36.0f);
        CHECK(fabsf(turns - 50.19518086f) < 0.0001f);
        CHECK(tick - lift_tick >= (stale_id5 ? 16u : 10u));
        pair_attempts++;
        if (pair_retry && pair_attempts < 3u) { return MOTORAPP_ERR_NOFB; }
        pair_tick = tick;
    }
    pairs++; pair.state = pairs >= 5u ? MOTORAPP_PAIR_MOVING : MOTORAPP_PAIR_DONE;
    pair.actual_turns[0]=turns; pair.actual_turns[1]=turns;
    return MOTORAPP_OK;
}
osStatus_t ChassisTask_RequestFrontMove(void)
{
    requests++;
    CHECK(syncs == 3u && pairs == 4u);
    CHECK(g_upstairs_debug.second_stair.status == 2u);
    if (reject || (retry && requests == 1u)) { return osErrorResource; }
    accepted++;
    return osOK;
}
uint32_t ChassisTask_PollReply(void)
{
    if (end_front_accepted != 0u) {
        uint32_t age = tick - end_front_tick;
        end_front_polls++;
        if (end_front_wait || mock_replied) { return 0u; }
        if (end_front_error && age >= 4u) { mock_replied = 1; return CHASSIS_TASK_ERROR_FLAG; }
        if ((mock_pause_fraction > 0.0f && mock_resumed && tick - mock_resume_tick >= 10u) ||
            (mock_pause_fraction == 0.0f && ((link_early_done && age >= 4u) || age >= 20u))) {
            mock_replied = 1; return CHASSIS_TASK_DONE_FLAG;
        }
        return 0u;
    }
    if (fourth_accepted != 0u) {
        fourth_polls++;
        if (fourth_polls < 4u || wait_fourth) { return 0u; }
        return fourth_error ? CHASSIS_TASK_ERROR_FLAG : CHASSIS_TASK_DONE_FLAG;
    }
    CHECK(accepted == 1u); polls++;
    if (polls < 4u) { return 0u; }
    return error_reply ? CHASSIS_TASK_ERROR_FLAG : CHASSIS_TASK_DONE_FLAG;
}
osStatus_t ChassisTask_RequestFrontMoveTurns(float left, float right)
{
    if (pairs == 6u) {
        uint32_t min_age = end_id5_stale ? 24u : (end_id1_fast ? 20u : (end_id1_active ? 18u : (end_id1_stale ? 16u : 12u)));
        CHECK(end_id1_accepted == 1u && axes[2].state == AXIS_IDLE && axes[2].traj.active == 0u);
        CHECK(axes[0].state == AXIS_IDLE && axes[0].traj.active == 0u);
        CHECK(fabsf(motors[0].angle_rad - axes[0].traj.target) < 0.00001f);
        CHECK(tick - end_id1_tick >= min_age);
        CHECK(fabsf(motors[2].angle_rad - axes[2].traj.target) < 0.00001f && motors[2].speed_rad_s == 0.0f);
        CHECK(left > 0.0f && right < 0.0f && fabsf(left + right) < 0.0001f);
        end_front_requests++;
        if (end_front_reject || (end_front_retry && end_front_requests < 3u)) { return osErrorResource; }
        mock_front_target[0] = left; mock_front_target[1] = right;
        end_front_accepted++; end_front_tick = tick;
        return osOK;
    }
    CHECK(g_upstairs_debug.third_stair.status == 2u && syncs == 5u && pairs == 5u);
    CHECK(left > 0.0f && right < 0.0f && fabsf(left + right) < 0.0001f);
    fourth_requests++;
    if (fourth_reject || (fourth_retry && fourth_requests < 3u)) { return osErrorResource; }
    mock_actual[0] = left; mock_actual[1] = right;
    fourth_accepted++; mock_sequence = 2u;
    return osOK;
}
uint32_t ChassisTask_GetFrontProgress(float actual[2])
{
    actual[0] = mock_actual[0]; actual[1] = mock_actual[1]; return mock_sequence;
}
osStatus_t ChassisTask_RequestFrontMoveWithPause(float left, float right, float fraction)
{
    CHECK(fabsf(fraction - 0.4f) < 0.00001f);
    mock_pause_fraction = fraction;
    return ChassisTask_RequestFrontMoveTurns(left, right);
}
osStatus_t ChassisTask_ResumeFrontMove(uint32_t sequence)
{
    CHECK(sequence == 3u && link_id2_accepted == 1u);
    CHECK(axes[1].state == AXIS_IDLE && axes[1].traj.active == 0u);
    CHECK(fabsf(motors[1].speed_rad_s) < 0.001f);
    if (tick - end_front_tick < 8u) { return osErrorResource; }
    CHECK(fabsf(motors[1].angle_rad - link_original_target) < 0.00001f);
    mock_resume_attempts++;
    if (resume_retry && mock_resume_attempts < 3u) { return osErrorResource; }
    mock_resume_requests++;
    CHECK(mock_resumed == 0);
    mock_resumed = 1; mock_resume_tick = tick;
    return osOK;
}
int main(int argc, char **argv)
{
    (void)argc;
    retry=strcmp(argv[1],"retry")==0;
    reject=strcmp(argv[1],"reject")==0;
    error_reply=strcmp(argv[1],"error")==0;
    lift_retry=strcmp(argv[1],"lift_retry")==0;
    lift_rejected=strcmp(argv[1],"lift_rejected")==0;
    wait_id5=strcmp(argv[1],"wait_id5")==0;
    wait_id1=strcmp(argv[1],"wait_id1")==0;
    stale_id5=strcmp(argv[1],"stale_id5")==0;
    lift_fault=strcmp(argv[1],"lift_fault")==0;
    pair_retry=strcmp(argv[1],"pair_retry")==0;
    pair_fault=strcmp(argv[1],"pair_fault")==0;
    wait_pair=strcmp(argv[1],"wait_pair")==0;
    final_retry=strcmp(argv[1],"final_retry")==0;
    final_rejected=strcmp(argv[1],"final_rejected")==0;
    final_fault=strcmp(argv[1],"final_fault")==0;
    wait_final=strcmp(argv[1],"wait_final")==0;
    wait_final_id5=strcmp(argv[1],"wait_final_id5")==0;
    final_fault_id5=strcmp(argv[1],"final_fault_id5")==0;
    stale_final_id5=strcmp(argv[1],"stale_final_id5")==0;
    fourth_retry=strcmp(argv[1],"fourth_retry")==0;
    fourth_reject=strcmp(argv[1],"fourth_reject")==0;
    fourth_error=strcmp(argv[1],"fourth_error")==0;
    wait_fourth=strcmp(argv[1],"wait_fourth")==0;
    fourth_lift_retry=strcmp(argv[1],"fourth_lift_retry")==0;
    fourth_lift_reject=strcmp(argv[1],"fourth_lift_reject")==0;
    fourth_wait_id5=strcmp(argv[1],"fourth_wait_id5")==0;
    fourth_wait_id1=strcmp(argv[1],"fourth_wait_id1")==0;
    fourth_stale_id5=strcmp(argv[1],"fourth_stale_id5")==0;
    fourth_lift_fault=strcmp(argv[1],"fourth_lift_fault")==0;
    fourth_pair_retry=strcmp(argv[1],"fourth_pair_retry")==0;
    fourth_pair_reject=strcmp(argv[1],"fourth_pair_reject")==0;
    fourth_pair_fault=strcmp(argv[1],"fourth_pair_fault")==0;
    fourth_wait_pair=strcmp(argv[1],"fourth_wait_pair")==0;
    end_id1_retry=strcmp(argv[1],"end_id1_retry")==0;
    end_id1_reject=strcmp(argv[1],"end_id1_reject")==0;
    end_id1_wait=strcmp(argv[1],"end_id1_wait")==0;
    end_id1_stale=strcmp(argv[1],"end_id1_stale")==0;
    end_id1_fast=strcmp(argv[1],"end_id1_fast")==0;
    end_id1_active=strcmp(argv[1],"end_id1_active")==0;
    end_id1_fault=strcmp(argv[1],"end_id1_fault")==0;
    end_front_retry=strcmp(argv[1],"end_front_retry")==0;
    end_front_reject=strcmp(argv[1],"end_front_reject")==0;
    end_front_wait=strcmp(argv[1],"end_front_wait")==0;
    end_front_error=strcmp(argv[1],"end_front_error")==0;
    end_id5_wait=strcmp(argv[1],"end_id5_wait")==0;
    end_id5_stale=strcmp(argv[1],"end_id5_stale")==0;
    end_id5_fault=strcmp(argv[1],"end_id5_fault")==0;
    link_id2_retry=strcmp(argv[1],"link_id2_retry")==0;
    link_id2_reject=strcmp(argv[1],"link_id2_reject")==0;
    link_id2_wait=strcmp(argv[1],"link_id2_wait")==0;
    link_id2_stale=strcmp(argv[1],"link_id2_stale")==0;
    link_id2_fast=strcmp(argv[1],"link_id2_fast")==0;
    link_id2_active=strcmp(argv[1],"link_id2_active")==0;
    link_id2_fault=strcmp(argv[1],"link_id2_fault")==0;
    link_early_done=strcmp(argv[1],"link_early_done")==0;
    link_front_wait_after_trigger=strcmp(argv[1],"link_front_wait_after_trigger")==0;
    if (link_front_wait_after_trigger) { end_front_wait = 1; }
    resume_retry=strcmp(argv[1],"resume_retry")==0;
    link_id2_reset=strcmp(argv[1],"link_id2_reset")==0;
    return_retry=strcmp(argv[1],"return_retry")==0;
    return_reject=strcmp(argv[1],"return_reject")==0;
    return_wait=strcmp(argv[1],"return_wait")==0;
    return_stale=strcmp(argv[1],"return_stale")==0;
    return_fast=strcmp(argv[1],"return_fast")==0;
    return_active=strcmp(argv[1],"return_active")==0;
    return_fault=strcmp(argv[1],"return_fault")==0;
    return_reset=strcmp(argv[1],"return_reset")==0;
    if (setjmp(stop)==0) { upstairs_task(); }
    if (reject) {
        CHECK(requests > 1u && accepted == 0u && polls == 0u);
        CHECK(g_upstairs_debug.state == 2u && g_upstairs_debug.step == 1u);
    } else {
        CHECK(requests == (retry ? 2u : 1u));
        CHECK(accepted == 1u && polls == 4u);
        if (error_reply) {
            CHECK(syncs == 3u && pairs == 4u && g_upstairs_debug.state == 3u);
        } else if (lift_rejected || lift_fault) {
            CHECK(pairs == 4u && g_upstairs_debug.state == 3u);
            CHECK(g_upstairs_debug.step == 2u);
        } else if (wait_id5 || wait_id1) {
            CHECK(syncs == 4u && pairs == 4u);
            CHECK(g_upstairs_debug.state == 2u && g_upstairs_debug.step == 2u);
        } else {
            CHECK(syncs >= (wait_pair || pair_fault || final_rejected ? 4u : 5u) && pairs >= 5u);
            if (wait_pair || pair_fault || wait_final || final_fault || final_rejected || wait_final_id5 || final_fault_id5) {
                CHECK(g_upstairs_debug.step == (wait_pair || pair_fault ? 3u : 4u));
                CHECK(g_upstairs_debug.state == (pair_fault || final_fault || final_rejected || final_fault_id5 ? 3u : 2u));
                CHECK(fourth_requests == 0u);
            } else {
                int lift_blocked = fourth_lift_reject || fourth_wait_id5 || fourth_wait_id1 || fourth_lift_fault;
                int pair_blocked = fourth_pair_reject || fourth_pair_fault || fourth_wait_pair;
                int end_blocked = end_id1_reject || end_id1_fault || end_id1_wait || end_front_reject || end_front_wait || end_front_error || end_id5_wait || end_id5_fault;
                int fourth_fault = fourth_error || fourth_lift_reject || fourth_lift_fault || fourth_pair_reject || fourth_pair_fault || end_id1_reject || end_id1_fault || end_front_error || end_id5_fault;
                int front_blocked = fourth_reject || fourth_error || wait_fourth;
                int fourth_blocked = front_blocked || lift_blocked || pair_blocked || end_blocked;
                int return_blocked = return_reject || return_wait || return_fault || return_reset;
                int no_link = front_blocked || lift_blocked || pair_blocked || end_id1_reject || end_id1_fault || end_id1_wait || end_id5_wait || end_id5_fault || end_front_reject || end_front_error || (end_front_wait && !link_front_wait_after_trigger);
                if (link_id2_reject || link_id2_wait || link_id2_fault || link_id2_reset) { fourth_blocked = 1; end_blocked = 1; }
                if (link_id2_reject || link_id2_fault) { fourth_fault = 1; }
                if (return_reject || return_fault) { fourth_fault = 1; }
                CHECK(g_upstairs_debug.state == (fourth_fault ? 3u : 4u));
                CHECK(g_upstairs_debug.step == (front_blocked ? 1u : (lift_blocked ? 2u : (pair_blocked ? 3u : (end_blocked ? 4u : (return_blocked ? 5u : 6u))))));
                if (no_link) { CHECK(link_id2_attempts == 0u); }
                else {
                    CHECK(link_id2_attempts == (link_id2_retry ? 3u : 1u));
                    CHECK(link_id2_accepted == (link_id2_reject ? 0u : 1u));
                    if (link_id2_accepted != 0u) {
                        CHECK(g_upstairs_debug.fourth_stair.id2_trigger_ms == (link_early_done ? 4u : (link_id2_retry ? 8u : 6u)));
                        CHECK(g_upstairs_debug.fourth_stair.front35_ms == (link_early_done ? UINT32_MAX : 6u));
                        if (g_upstairs_debug.fourth_stair.front75_ms != UINT32_MAX) {
                            uint8_t met = (uint8_t)(g_upstairs_debug.fourth_stair.id2_done_ms < g_upstairs_debug.fourth_stair.front75_ms);
                            CHECK(g_upstairs_debug.fourth_stair.id2_before75 == met);
                            CHECK(g_upstairs_debug.fourth_stair.id2_deadline_missed == (uint8_t)!met);
                        }
                        if (g_upstairs_debug.fourth_stair.id2_done_ms != UINT32_MAX) {
                            uint32_t duration = link_id2_fast ? 20u : (link_id2_active ? 18u : (link_id2_stale ? 16u : 8u));
                            CHECK(g_upstairs_debug.fourth_stair.id2_done_ms == g_upstairs_debug.fourth_stair.id2_trigger_ms + duration);
                        }
                    }
                    if (!fourth_blocked) { CHECK(axes[1].traj.v_max == 30.0f && axes[1].traj.a_max == 30.0f); }
                }
                CHECK(g_upstairs_debug.fourth_stair.status == (fourth_fault ? 3u : (fourth_blocked || return_blocked ? 1u : 2u)));
                if (end_front_accepted != 0u) { CHECK(fabsf(mock_pause_fraction - 0.4f) < 0.00001f); }
                if (link_id2_wait || link_id2_reset) {
                    CHECK(mock_resumed == 0 && mock_resume_requests == 0u);
                    CHECK(mock_actual[0] <= mock_front_target[0] * 0.4001f);
                    if (link_id2_reset) { CHECK(fabsf(g_upstairs_debug.fourth_stair.id2_error_turns - 3.84f) < 0.0001f); }
                }
                if (front_blocked) { CHECK(fourth_lift_attempts == 0u && fourth_pair_attempts == 0u); }
                else {
                    CHECK(fourth_lift_attempts == (fourth_lift_retry ? 3u : 1u));
                    if (lift_blocked) { CHECK(fourth_pair_attempts == 0u && pairs == 5u); }
                    else {
                        CHECK(fourth_pair_attempts == (fourth_pair_retry ? 3u : 1u));
                        if (!pair_blocked) {
                            CHECK(pairs == 6u && pair.state == MOTORAPP_PAIR_DONE);
                            CHECK(g_upstairs_debug.fourth_stair.lift_move_result == MOTORAPP_OK);
                            CHECK(g_upstairs_debug.fourth_stair.pair_move_result == MOTORAPP_OK);
                            CHECK(g_upstairs_debug.fourth_stair.step2_ms >= (fourth_stale_id5 ? 18u : 12u));
                            CHECK(g_upstairs_debug.fourth_stair.step3_ms >= 7u);
                            CHECK(end_id1_attempts == (end_id1_retry ? 3u : 1u));
                            if (end_id1_reject || end_id1_fault || end_id1_wait || end_id5_wait || end_id5_fault) { CHECK(end_front_requests == 0u); }
                            else {
                                CHECK(end_id1_accepted == 1u);
                                if (end_front_reject) { CHECK(end_front_requests > 1u && end_front_accepted == 0u); }
                                else {
                                    CHECK(end_front_requests == (end_front_retry ? 3u : 1u));
                                    CHECK(end_front_accepted == 1u && end_front_polls >= 4u);
                                }
                            }
                            if (!end_blocked) {
                                CHECK(return_attempts == (return_retry ? 3u : 1u));
                                CHECK(return_accepted == (return_reject ? 0u : 1u));
                                if (return_reset) { CHECK(fabsf(g_upstairs_debug.fourth_stair.id1_return_error_turns - 35.0f) < 0.0001f); }
                                CHECK(mock_resume_requests == 1u);
                                CHECK(mock_resume_attempts == (resume_retry ? 3u : 1u));
                                CHECK(g_upstairs_debug.fourth_stair.front_resume_requested == 1u);
                                CHECK(g_upstairs_debug.fourth_stair.id1_move_result == MOTORAPP_OK);
                                CHECK(g_upstairs_debug.fourth_stair.id5_move_result == MOTORAPP_OK);
                                CHECK(g_upstairs_debug.fourth_stair.front_final_request_result == osOK);
                                CHECK(g_upstairs_debug.fourth_stair.front_final_reply == CHASSIS_TASK_DONE_FLAG);
                                CHECK(end_front_polls >= (link_early_done ? 4u : 20u));
                                CHECK(g_upstairs_debug.fourth_stair.total_ms ==
                                    g_upstairs_debug.fourth_stair.step1_ms + g_upstairs_debug.fourth_stair.step2_ms +
                                    g_upstairs_debug.fourth_stair.step3_ms + g_upstairs_debug.fourth_stair.step4_ms +
                                    g_upstairs_debug.fourth_stair.step5_ms);
                                if (!return_blocked) {
                                    CHECK(axes[2].state == AXIS_IDLE && axes[2].traj.active == 0u);
                                    CHECK(fabsf(motors[2].angle_rad - return_original_target) < 0.00001f);
                                    CHECK(g_upstairs_debug.fourth_stair.step5_ms >= (return_fast ? 20u : (return_active ? 18u : (return_stale ? 16u : 8u))));
                                    CHECK(g_upstairs_debug.fourth_stair.id1_return_move_result == MOTORAPP_OK);
                                    CHECK(g_upstairs_debug.fourth_stair.total_ms < tick - g_upstairs_debug.fourth_stair.step1_start_ms);
                                }
                            }
                        }
                    }
                }
                if (fourth_reject) {
                    CHECK(fourth_requests > 1u && fourth_accepted == 0u && fourth_polls == 0u);
                } else {
                    CHECK(fourth_requests == (fourth_retry ? 3u : 1u) && fourth_accepted == 1u);
                    if (wait_fourth) { CHECK(fourth_polls > 4u); }
                    else { CHECK(fourth_polls == 4u); }
                    CHECK(g_upstairs_debug.fourth_stair.request_result == osOK);
                }
            }
            CHECK(lift_attempts == (lift_retry ? 3u : 1u));
            CHECK(pair_attempts == (pair_retry ? 3u : 1u));
            CHECK(g_upstairs_debug.third_stair.status == (pair_fault || final_fault || final_rejected || final_fault_id5 ?
                3u : (wait_pair || wait_final || wait_final_id5 ? 1u : 2u)));
            if (!pair_fault && !wait_pair && !final_fault && !final_rejected && !wait_final &&
                !final_fault_id5 && !wait_final_id5) {
                CHECK(final_count == 1u && final_attempts == (final_retry ? 3u : 1u));
                if (syncs == 5u) {
                    CHECK(axes[0].state == AXIS_IDLE && axes[0].traj.active == 0u);
                    CHECK(axes[2].state == AXIS_IDLE && axes[2].traj.active == 0u);
                }
                CHECK(g_upstairs_debug.third_stair.total_ms ==
                    g_upstairs_debug.third_stair.step1_ms + g_upstairs_debug.third_stair.step2_ms +
                    g_upstairs_debug.third_stair.step3_ms + g_upstairs_debug.third_stair.step4_ms);
                CHECK(g_upstairs_debug.third_stair.lift_move_result == MOTORAPP_OK);
                CHECK(g_upstairs_debug.third_stair.pair_move_result == MOTORAPP_OK);
                CHECK(g_upstairs_debug.third_stair.step2_ms >= 10u);
                CHECK(g_upstairs_debug.third_stair.step3_ms >= 8u);
                CHECK(g_upstairs_debug.third_stair.step4_ms >= (stale_final_id5 ? 14u : 8u));
                CHECK(g_upstairs_debug.third_stair.id1_move_result == MOTORAPP_OK);
                CHECK(g_upstairs_debug.third_stair.id5_move_result == MOTORAPP_OK);
            }
            if (wait_pair || pair_fault) { CHECK(final_attempts == 0u); }
            if (final_rejected) { CHECK(final_attempts == 1u && final_count == 0u); }
            if (wait_final || final_fault) { CHECK(final_attempts == 1u && final_count == 1u); }
            if (wait_final_id5 || final_fault_id5) { CHECK(final_attempts == 1u && final_count == 1u); }
        }
    }
    printf("Third stair task %s: %u failures\n",argv[1],failures);
    return failures==0u?0:1;
}
