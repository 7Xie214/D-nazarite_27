/* First-stair ordering and second-stair first move. Full second stair has a separate fixture. */
#include "My_task.h"
#include "MotorApp.h"
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

extern volatile uint8_t g_upstairs_start;
static Axis_t axes[5];
static DjiMotor_t motors[5];
static const DjiMotor_Model_t model = {.gear_ratio = 3591.0f / 187.0f};
static const DjiMotor_Model_t model_2006 = {.gear_ratio = 36.0f};
static uint32_t tick;
static uint32_t timing_offset;
static uint32_t step1_start_tick, step2_start_tick;
static uint32_t step2_end_tick, step4_start_tick, step4_end_tick, timing_fault_tick;
static unsigned int failures, updates, starts, single_starts[5], mode;
static uint32_t sync_started, single_started[5];
static unsigned int pair_starts;
static uint32_t pair_started[2];
static unsigned int half_attempts;
static unsigned int position_handoff;
static unsigned int arrival_fallback;
static unsigned int step2_diag_case;
static unsigned int second_fault_case, second_retry_case, second_attempts;
static uint32_t second_end_tick, second_fault_tick;
static float pair_target_turns;
static MotorApp_2006PairStatus_t pair_status;
static jmp_buf finish;
#define CHECK(x) do { if (!(x)) { printf("FAIL %d: %s\n", __LINE__, #x); failures++; } } while (0)
uint32_t osKernelGetTickCount(void) { return tick; }
uint32_t HAL_GetTick(void) { return tick + timing_offset; }
void MotorApp_Init(void)
{
    static const uint8_t ids[5] = {5u, 2u, 1u, 4u, 3u};
    unsigned int i;
    memset(axes, 0, sizeof(axes));
    memset(motors, 0, sizeof(motors));
    memset(&pair_status, 0, sizeof(pair_status));
    for (i = 0u; i < 5u; i++) {
        axes[i].motor = &motors[i];
        motors[i].id = ids[i];
        motors[i].model = i < 3u ? &model : &model_2006;
        motors[i].rx_count = 100u + i;
    }
    axes[1].traj.target = 1.0f;
    axes[1].settled_s = 0.012f;
    motors[1].angle_rad = 0.5f;
    motors[1].raw_speed = -8;
    motors[1].cmd = -1234;
    axes[3].traj.target = 1.0f;
    motors[3].raw_current = -450;
    motors[3].total_counts = 123456;
}
Axis_t *MotorApp_GetAxis(uint8_t i) { return i < 5u ? &axes[i] : NULL; }
const MotorApp_2006PairStatus_t *MotorApp_Get2006PairStatus(void) { return &pair_status; }
void MotorApp_Update(float dt) { CHECK(dt > 0.0009f && dt < 0.0011f); updates++; }
int MotorApp_Move3508TurnsSync(const float turns[3], float rate, float accel)
{
    unsigned int i;
    CHECK(turns != NULL && rate > 0.0f && accel > 0.0f);
    CHECK(mode != 0u && tick >= 5u);
    if (tick < 10u) { return MOTORAPP_ERR_NOFB; }
    CHECK(starts < 3u);
    if (starts == 2u) {
        CHECK(g_upstairs_debug.first_stair_done == 1u && pair_starts == 2u);
        CHECK(turns[0] == 35.0f && turns[1] == 0.0f && turns[2] == -35.0f);
        second_attempts++;
        for (i = 0u; i < 3u; i++) {
            if (axes[i].state == AXIS_MOVING) { return MOTORAPP_ERR_BUSY; }
        }
        if (second_retry_case != 0u && tick < 150u) { return MOTORAPP_ERR_NOFB; }
    }
    if (starts == 0u) {
        CHECK(turns[0] == 0.0f); /* Step 1 holds ID5, moves ID2 / ID1. */
        CHECK(turns[2] != 0.0f);
    } else if (starts == 1u) {
        CHECK(tick > sync_started + 30u); /* Wait for ALL first-step axes. */
    }
    {
        Traj_t progress;
        unsigned int steps;
        Traj_Init(&progress, rate, accel);
        unsigned int has_travel = turns[0] != 0.0f || turns[1] != 0.0f || turns[2] != 0.0f;
        Traj_Start(&progress, 0.0f, has_travel != 0u ? 1.0f : 0.0f);
        for (steps = 0u; steps < 12000u && Traj_IsActive(&progress) != 0; steps++) {
            Traj_Update(&progress, 0.001f);
        }
        printf("Task reference duration: %.3f s\n", (double)steps * 0.001);
        CHECK(has_travel != 0u ? (steps >= 750u && steps <= 850u) : steps == 1u);
    }
    starts++;
    sync_started = tick;
    for (i = 0u; i < 3u; i++) { axes[i].state = AXIS_MOVING; }
    return MOTORAPP_OK;
}
int MotorApp_Move2006Pair(float turns)
{
    unsigned int i;
    CHECK(turns > 0.0f);
    CHECK(starts == 2u && tick > sync_started + 30u);
    CHECK(mode == 3u || mode == 4u || mode == 7u || mode == 8u || mode == 9u || mode == 10u);
    if (mode == 4u && tick < 90u) { return MOTORAPP_ERR_NOFB; }
    if (pair_starts == 0u) {
        CHECK(fabsf(turns - 50.19518086f) < 1e-4f);
    } else {
        CHECK(turns > 0.0f); /* Additional travel remains user-configurable. */
        CHECK(pair_status.state == MOTORAPP_PAIR_DONE);
        CHECK(single_starts[0] == 1u);
        if (mode == 4u) { CHECK(axes[0].state == AXIS_IDLE); }
        else {
            CHECK(axes[0].state == AXIS_MOVING && axes[0].traj.active != 0u);
            CHECK(fabsf(axes[0].traj.target - motors[0].angle_rad) > 0.1f);
            CHECK(tick < single_started[0] + 20u); /* pair completion does not wait for ID5 */
        }
    }
    pair_starts++;
    CHECK(pair_starts <= 2u);
    pair_started[pair_starts - 1u] = HAL_GetTick();
    memset(&pair_status.timing, 0, sizeof(pair_status.timing));
    pair_status.timing.start_ms = HAL_GetTick();
    pair_status.timing.status = 1u;
    pair_status.timing.planner_ms = 1000u + pair_starts; /* distinguish the saved legs */
    for (i = 3u; i < 5u; i++) {
        single_starts[i]++;
        single_started[i] = tick;
        axes[i].state = AXIS_MOVING;
    }
    pair_status.state = MOTORAPP_PAIR_MOVING;
    pair_target_turns = turns;
    return MOTORAPP_OK;
}
int MotorApp_MoveTurns(Axis_t *axis, float turns)
{
    CHECK(axis == &axes[0]);
    CHECK(turns < 0.0f); /* User-tunable return travel: preserve current magnitude. */
    CHECK(pair_status.state != MOTORAPP_PAIR_FAULT);
    if (pair_status.state == MOTORAPP_PAIR_MOVING) {
        CHECK(tick >= single_started[3] + 3u);
        CHECK(pair_status.actual_turns[0] + 1e-4f >= pair_target_turns * 0.75f);
        CHECK(pair_status.actual_turns[1] + 1e-4f >= pair_target_turns * 0.75f);
    } else { CHECK(pair_status.state == MOTORAPP_PAIR_DONE); }
    if (arrival_fallback != 0u && half_attempts == 0u) {
        CHECK(pair_status.state == MOTORAPP_PAIR_DONE);
        CHECK(pair_status.actual_turns[1] < pair_target_turns * 0.75f);
    }
    half_attempts++;
    if (mode == 8u && tick < single_started[3] + 8u) { return MOTORAPP_ERR_BUSY; }
    CHECK(single_starts[0] == 0u);
    single_starts[0]++;
    single_started[0] = tick;
    axis->state = AXIS_MOVING;
    axis->traj.target = axis->motor->angle_rad + turns * 6.28318530718f / axis->motor->model->gear_ratio;
    axis->traj.active = 1u;
    return MOTORAPP_OK;
}
void osDelay(uint32_t ms)
{
    CHECK(ms == 1u);
    tick++;
    if (mode != 0u) {
        if (step1_start_tick == 0u && g_upstairs_debug.timing.status == 1u) {
            step1_start_tick = tick - 1u;
            CHECK(step1_start_tick == 5u); /* exclude debug-start waiting */
            CHECK(g_upstairs_debug.timing.step1_start_ms == step1_start_tick + timing_offset);
        }
        if (step2_start_tick == 0u && g_upstairs_debug.state == 0u &&
            g_upstairs_debug.step == 2u && g_upstairs_debug.timing.status == 1u) {
            step2_start_tick = tick - 1u;
            CHECK(g_upstairs_debug.timing.step2_start_ms == step2_start_tick + timing_offset);
        }
        if (step2_end_tick == 0u && g_upstairs_debug.step == 3u && g_upstairs_debug.timing.status == 1u) {
            step2_end_tick = tick - 1u;
            CHECK(g_upstairs_debug.step == 3u);
            CHECK(g_upstairs_debug.timing.step2_end_ms == step2_end_tick + timing_offset);
        }
        if (step4_start_tick == 0u && g_upstairs_debug.step == 4u) {
            step4_start_tick = tick - 1u;
            CHECK(g_upstairs_debug.timing.step4_start_ms == step4_start_tick + timing_offset);
        }
        if (step4_end_tick == 0u && g_upstairs_debug.first_stair_done != 0u) {
            step4_end_tick = tick - 1u;
            CHECK(g_upstairs_debug.timing.step4_end_ms == step4_end_tick + timing_offset);
        }
        if (timing_fault_tick == 0u && g_upstairs_debug.timing.status == 3u) {
            timing_fault_tick = tick - 1u;
        }
        if (second_end_tick == 0u && g_upstairs_debug.state == 1u && g_upstairs_debug.step == 2u) {
            second_end_tick = tick - 1u;
        }
        if (second_fault_tick == 0u && g_upstairs_debug.second_stair.status == 3u) {
            second_fault_tick = tick - 1u;
        }
        if (mode == 4u && tick == 85u) {
            CHECK(g_upstairs_debug.move_2006_result[0] == MOTORAPP_ERR_NOFB);
            CHECK(g_upstairs_debug.move_2006_result[1] == MOTORAPP_ERR_NOFB);
            CHECK(single_starts[3] == 0u && single_starts[4] == 0u);
        }
        if (tick == 5u || tick == 15u) { g_upstairs_start = 1u; }
        if (tick == 6u) { g_upstairs_start = 0u; }
        if (starts != 0u) {
            if (tick == sync_started + 10u) { axes[0].state = AXIS_IDLE; }
            if (tick == sync_started + 20u) { axes[1].state = AXIS_IDLE; }
            if (tick == sync_started + 30u) { axes[2].state = AXIS_IDLE; }
            if (((mode == 2u && starts == 1u) || (mode == 5u && starts == 2u)) &&
                tick == sync_started + 15u) { axes[1].state = AXIS_FAULT; }
            if (second_fault_case != 0u && starts == 3u && tick == sync_started + 15u) {
                axes[1].state = AXIS_FAULT;
            }
            if (step2_diag_case != 0u && starts == 2u && pair_starts == 0u) {
                unsigned int j;
                uint32_t age = tick - sync_started;
                for (j = 0u; j < 3u; j++) {
                    axes[j].state = age < (j == 2u ? 30u : 20u) ? AXIS_MOVING : AXIS_IDLE;
                    axes[j].traj.active = age < 5u ? 1u : 0u;
                    axes[j].traj.target = motors[j].angle_rad;
                    if (j < 2u && age < (j == 0u ? 10u : 20u)) {
                        axes[j].traj.target += 0.2f * 6.28318530718f / motors[j].model->gear_ratio;
                    }
                    motors[j].speed_rad_s = j != 1u && age >= (j == 0u ? 10u : 5u) &&
                        age < (j == 0u ? 20u : 30u) ? 1.0f : 0.0f;
                    if (j != 1u) { motors[j].raw_speed = motors[j].speed_rad_s > 0.0f ? 183 : 0; }
                }
            }
        }
        if (mode == 3u || mode == 4u || mode == 8u || mode == 9u || mode == 10u) {
            unsigned int i;
            for (i = 3u; i < 5u; i++) {
                if (!(mode == 10u && pair_starts == 2u) && single_started[i] != 0u &&
                    tick == single_started[i] + (mode == 4u ? 35u : 10u)) { axes[i].state = AXIS_IDLE; }
            }
            if (!(mode == 10u && pair_starts == 2u) && single_started[3] != 0u &&
                tick == single_started[3] + (mode == 4u ? 35u : 10u)) {
                pair_status.state = MOTORAPP_PAIR_DONE;
            }
            if (single_started[3] != 0u) {
                uint32_t elapsed = tick - single_started[3];
                pair_status.reference_turns = pair_target_turns;
                pair_status.actual_turns[0] = pair_target_turns * (elapsed >= 10u ? 1.0f : (elapsed >= 2u ? 0.8f : 0.0f));
                pair_status.actual_turns[1] = pair_target_turns * (elapsed >= 10u ? 1.0f : (elapsed >= 3u ? 0.8f : 0.74f));
                if (elapsed == 3u) {
                    pair_status.actual_turns[0] = pair_target_turns * 0.75f;
                    pair_status.actual_turns[1] = pair_target_turns * 0.75f;
                }
                if (arrival_fallback != 0u && pair_starts == 1u && elapsed == 1u) {
                    pair_status.actual_turns[0] = pair_target_turns;
                    pair_status.actual_turns[1] = pair_target_turns * 0.2f;
                    pair_status.state = MOTORAPP_PAIR_DONE;
                    axes[3].state = AXIS_IDLE; axes[4].state = AXIS_IDLE;
                }
                if (elapsed == 7u) {
                    pair_status.actual_turns[0] = pair_target_turns * 0.3f;
                    pair_status.actual_turns[1] = pair_target_turns * 0.3f;
                } /* retreat across halfway must not reissue an accepted move */
                if (elapsed == 3u && pair_starts == 1u) { CHECK(single_starts[0] == 0u); } /* one wheel/reference is insufficient */
                if (elapsed == 12u && pair_starts == 1u) {
                    CHECK(g_upstairs_debug.step == 3u && g_upstairs_debug.state == 0u);
                } /* pair done alone must not advance while ID5 moves */
                if (mode == 4u && elapsed == 25u && pair_starts == 1u) {
                    CHECK(axes[0].state == AXIS_IDLE && pair_status.state == MOTORAPP_PAIR_MOVING);
                    CHECK(g_upstairs_debug.step == 3u && g_upstairs_debug.state == 0u);
                } /* ID5 done alone must also wait for the pair */
            }
            if (single_started[0] != 0u && tick == single_started[0] + 20u) {
                motors[0].angle_rad = axes[0].traj.target;
                if (position_handoff != 0u) { motors[0].raw_speed = 100; }
                else { axes[0].state = AXIS_IDLE; }
            }
        }
        if (mode == 7u && single_started[3] != 0u && tick == single_started[3] + 5u) {
            pair_status.state = MOTORAPP_PAIR_FAULT;
            pair_status.fault = MOTORAPP_PAIR_FAULT_OFFLINE;
        }
        if (mode == 9u && pair_starts == 2u && tick == single_started[3] + 5u) {
            pair_status.state = MOTORAPP_PAIR_FAULT;
            pair_status.fault = MOTORAPP_PAIR_FAULT_OFFLINE;
        }
        if (pair_starts != 0u && pair_status.timing.status == 1u) {
            pair_status.timing.total_ms = HAL_GetTick() - pair_status.timing.start_ms;
            if (pair_status.state == MOTORAPP_PAIR_DONE) { pair_status.timing.status = 2u; }
            if (pair_status.state == MOTORAPP_PAIR_FAULT) { pair_status.timing.status = 3u; }
        }
    }
    if ((g_upstairs_debug.state == 1u && g_upstairs_debug.step == 2u && g_upstairs_debug.issued == 0u) ||
        tick == (mode >= 3u ? 220u : 60u)) { longjmp(finish, 1); }
}
int main(int argc, char **argv)
{
    (void)argc;
    position_handoff = strcmp(argv[1], "id5_position") == 0 ? 1u : 0u;
    arrival_fallback = (strcmp(argv[1], "arrival_fallback") == 0 || strcmp(argv[1], "fallback_retry") == 0) ? 1u : 0u;
    step2_diag_case = (strcmp(argv[1], "step2_diag") == 0 || strcmp(argv[1], "step2_diag_wrap") == 0) ? 1u : 0u;
    timing_offset = (strcmp(argv[1], "timing_wrap") == 0 || strcmp(argv[1], "step2_diag_wrap") == 0) ? 0xffffffb0u : 0u;
    mode = strcmp(argv[1], "wait") == 0 ? 0u :
           (strcmp(argv[1], "start") == 0 ? 1u : ((strcmp(argv[1], "flow") == 0 || position_handoff != 0u || timing_offset != 0u) ? 3u :
            (strcmp(argv[1], "late_2006") == 0 ? 4u : (strcmp(argv[1], "fault_step2") == 0 ? 5u :
             (strcmp(argv[1], "first_only") == 0 ? 6u : (strcmp(argv[1], "fault_pair") == 0 ? 7u :
              (strcmp(argv[1], "half_retry") == 0 ? 8u : (strcmp(argv[1], "fault_final") == 0 ? 9u :
               (strcmp(argv[1], "wait_final") == 0 ? 10u : 2u)))))))));
    if (arrival_fallback != 0u) { mode = strcmp(argv[1], "fallback_retry") == 0 ? 8u : 3u; }
    if (step2_diag_case != 0u) { mode = 3u; }
    second_fault_case = strcmp(argv[1], "second_fault") == 0 ? 1u : 0u;
    second_retry_case = strcmp(argv[1], "second_retry") == 0 ? 1u : 0u;
    if (second_fault_case != 0u || second_retry_case != 0u) { mode = 3u; }
    CHECK(g_upstairs_start == 0u);
    if (setjmp(finish) == 0) { upstairs_task(); }
    CHECK(updates >= 50u);
    CHECK(starts == (mode == 0u ? 0u : (mode == 2u || mode == 6u ? 1u :
          ((mode == 3u || mode == 4u || mode == 8u) && position_handoff == 0u ? 3u : 2u))));
    CHECK(pair_starts == (mode == 3u || mode == 4u || mode == 8u || mode == 9u || mode == 10u ? 2u : (mode == 7u ? 1u : 0u)));
    CHECK(single_starts[3] == pair_starts && single_starts[4] == pair_starts);
    CHECK(single_starts[0] == (mode == 3u || mode == 4u || mode == 8u || mode == 9u || mode == 10u ? 1u : 0u));
    CHECK(g_upstairs_debug.id5_half_started == single_starts[0]);
    CHECK(g_upstairs_debug.first_stair_done == (mode == 3u || mode == 4u || mode == 8u ? 1u : 0u));
    if (step2_diag_case != 0u) {
        CHECK(g_upstairs_debug.step2_axis[0].motor_id == 5u && g_upstairs_debug.step2_axis[2].motor_id == 1u);
        CHECK(g_upstairs_debug.step2_axis[0].planner_ms == 4u);
        CHECK(g_upstairs_debug.step2_axis[0].position_ms == 5u && g_upstairs_debug.step2_axis[0].speed_ms == 10u);
        CHECK(g_upstairs_debug.step2_axis[0].last_reason == 3u);
        CHECK(g_upstairs_debug.step2_axis[1].position_ms == 15u && g_upstairs_debug.step2_axis[1].speed_ms == 0u);
        CHECK(g_upstairs_debug.step2_axis[1].last_reason == 2u);
        CHECK(g_upstairs_debug.step2_axis[2].speed_ms == 25u && g_upstairs_debug.step2_axis[2].position_ms == 0u);
        CHECK(g_upstairs_debug.step2_axis[0].done_ms == 21u && g_upstairs_debug.step2_axis[2].done_ms == 31u);
        CHECK(g_upstairs_debug.step2_axis[2].last_speed_rpm == 183);
    }
    if (pair_starts != 0u) {
        CHECK(g_upstairs_debug.pair_timing[0].start_ms == pair_started[0]);
        CHECK(g_upstairs_debug.pair_timing[0].planner_ms == 1001u);
        CHECK(g_upstairs_debug.pair_timing[0].status == (mode == 7u ? 3u : 2u));
    }
    if (pair_starts == 2u) {
        CHECK(g_upstairs_debug.pair_timing[1].start_ms == pair_started[1]);
        CHECK(g_upstairs_debug.pair_timing[1].planner_ms == 1002u);
        CHECK(g_upstairs_debug.pair_timing[1].total_ms == g_upstairs_debug.pair_2006.timing.total_ms);
    }
    if (mode != 0u) {
        uint32_t stop_tick = g_upstairs_debug.timing.status == 2u ? step4_end_tick :
                             (g_upstairs_debug.timing.status == 3u ? timing_fault_tick : tick - 1u);
        CHECK(step1_start_tick == 5u);
        CHECK(g_upstairs_debug.timing.total_ms == stop_tick - step1_start_tick);
        CHECK(g_upstairs_debug.timing.step1_ms ==
              (step2_start_tick != 0u ? step2_start_tick : stop_tick) - step1_start_tick);
        if (step2_start_tick != 0u) {
            CHECK(g_upstairs_debug.timing.step2_ms ==
                  (step2_end_tick != 0u ? step2_end_tick : stop_tick) - step2_start_tick);
        }
    } else {
        CHECK(g_upstairs_debug.timing.total_ms == 0u && g_upstairs_debug.timing.step1_ms == 0u);
    }
    if (mode == 3u || mode == 4u || mode == 8u) {
        CHECK(g_upstairs_debug.timing.status == 2u);
        CHECK(step2_end_tick != 0u && step4_start_tick > step2_end_tick && step4_end_tick > step4_start_tick);
        CHECK(g_upstairs_debug.timing.elapsed_ms == step4_end_tick - step2_end_tick);
        CHECK(g_upstairs_debug.timing.step3_ms == step4_start_tick - step2_end_tick);
        CHECK(g_upstairs_debug.timing.step4_ms == step4_end_tick - step4_start_tick);
        CHECK(g_upstairs_debug.timing.elapsed_ms == g_upstairs_debug.timing.step3_ms + g_upstairs_debug.timing.step4_ms);
        CHECK(g_upstairs_debug.timing.step3_after_pair_ms == step4_start_tick + timing_offset -
              g_upstairs_debug.pair_timing[0].start_ms - g_upstairs_debug.pair_timing[0].total_ms);
        CHECK(g_upstairs_debug.timing.total_ms == g_upstairs_debug.timing.step1_ms +
              g_upstairs_debug.timing.step2_ms + g_upstairs_debug.timing.step3_ms +
              g_upstairs_debug.timing.step4_ms);
    } else if (mode == 7u || mode == 9u) {
        CHECK(g_upstairs_debug.timing.status == 3u && timing_fault_tick != 0u);
        CHECK(g_upstairs_debug.timing.elapsed_ms == timing_fault_tick - step2_end_tick);
        CHECK(g_upstairs_debug.timing.step4_end_ms == 0u);
    } else if (mode == 10u) {
        CHECK(g_upstairs_debug.timing.status == 1u);
        CHECK(g_upstairs_debug.timing.elapsed_ms == tick - 1u - step2_end_tick);
        CHECK(g_upstairs_debug.timing.step4_ms == tick - 1u - step4_start_tick);
        CHECK(g_upstairs_debug.timing.step4_end_ms == 0u);
    } else if (mode == 2u || mode == 5u) {
        CHECK(g_upstairs_debug.timing.status == 3u && timing_fault_tick != 0u);
        CHECK(g_upstairs_debug.timing.elapsed_ms == 0u);
    } else if (mode == 1u) {
        CHECK(g_upstairs_debug.timing.status == 1u && g_upstairs_debug.timing.elapsed_ms == 0u);
    } else {
        CHECK(g_upstairs_debug.timing.status == 0u && g_upstairs_debug.timing.elapsed_ms == 0u);
    }
    CHECK(g_upstairs_debug.move_id5_result == (single_starts[0] != 0u ? MOTORAPP_OK : 1));
    if (mode == 8u) { CHECK(half_attempts > 1u); }
    CHECK(g_upstairs_debug.updates == updates);
    CHECK(g_upstairs_debug.started == (mode == 0u ? 0u : 1u));
    CHECK(g_upstairs_debug.axis[0].motor_id == 5u);
    CHECK(g_upstairs_debug.axis[1].motor_id == 2u);
    CHECK(g_upstairs_debug.axis[2].motor_id == 1u);
    CHECK(g_upstairs_debug.axis[3].motor_id == 4u);
    CHECK(g_upstairs_debug.axis[4].motor_id == 3u);
    CHECK(g_upstairs_debug.axis[3].feedback_current == -450);
    CHECK(g_upstairs_debug.axis[3].total_counts == 123456);
    CHECK(fabsf(g_upstairs_debug.axis[3].error_turns - 5.729578f) < 1e-5f);
    CHECK(g_upstairs_debug.move_2006_result[0] == (pair_starts != 0u ? MOTORAPP_OK : 1));
    CHECK(g_upstairs_debug.move_2006_result[1] == (pair_starts != 0u ? MOTORAPP_OK : 1));
    CHECK(g_upstairs_debug.pair_2006.state == pair_status.state);
    CHECK(g_upstairs_debug.axis[1].state == (uint8_t)axes[1].state);
    CHECK(g_upstairs_debug.axis[1].planner_active == axes[1].traj.active);
    CHECK(g_upstairs_debug.axis[1].rx_count == 101u);
    CHECK(g_upstairs_debug.axis[1].command == -1234);
    CHECK(g_upstairs_debug.axis[1].speed_rpm == -8);
    CHECK(fabsf(g_upstairs_debug.axis[1].error_turns - (step2_diag_case != 0u ? 0.0f : 1.5281428f)) < 1e-5f);
    CHECK(fabsf(g_upstairs_debug.axis[1].settled_ms - 12.0f) < 1e-5f);
    if (mode == 0u) { CHECK(g_upstairs_debug.state == 0u && g_upstairs_debug.step == 1u && g_upstairs_debug.issued == 0u); }
    if (mode == 1u) { CHECK(g_upstairs_debug.state == 0u && g_upstairs_debug.step == 2u && g_upstairs_debug.issued == 1u); }
    if (mode == 2u || mode == 5u || mode == 7u || mode == 9u) { CHECK(g_upstairs_debug.state == 3u); }
    if (mode == 3u || mode == 4u || mode == 8u) {
        CHECK(g_upstairs_debug.state == (second_fault_case != 0u ? 3u : 1u));
        CHECK(g_upstairs_debug.second_stair.step1_start_ms == step4_end_tick + timing_offset);
        if (second_fault_case != 0u) {
            CHECK(g_upstairs_debug.second_stair.status == 3u && second_fault_tick != 0u);
            CHECK(g_upstairs_debug.second_stair.step1_ms == second_fault_tick - step4_end_tick);
        } else if (position_handoff != 0u) {
            CHECK(starts == 2u && g_upstairs_debug.second_stair.status == 1u);
            CHECK(g_upstairs_debug.second_stair.move_result == MOTORAPP_ERR_BUSY && second_attempts > 1u);
            CHECK(g_upstairs_debug.second_stair.step1_ms == tick - 1u - step4_end_tick);
        } else {
            CHECK(g_upstairs_debug.step == 2u && g_upstairs_debug.issued == 0u);
            CHECK(g_upstairs_debug.second_stair.status == 1u && second_end_tick != 0u);
            CHECK(g_upstairs_debug.second_stair.step1_ms == second_end_tick - step4_end_tick);
            CHECK(g_upstairs_debug.second_stair.move_result == MOTORAPP_OK);
        }
        if (second_retry_case != 0u) { CHECK(second_attempts > 1u && sync_started == 150u); }
    }
    if (mode == 6u) { CHECK(g_upstairs_debug.state == 0u && g_upstairs_debug.step == 2u && g_upstairs_debug.issued == 0u); }
    if (mode == 10u) { CHECK(g_upstairs_debug.state == 0u && g_upstairs_debug.step == 4u); }
    printf("Upstairs task %s: %u failures\n", argv[1], failures);
    return failures == 0u ? 0 : 1;
}
