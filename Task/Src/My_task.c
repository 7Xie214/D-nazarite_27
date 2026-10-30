/** 上楼机构任务：1kHz 更新电机，调试启动标志只放行一次流程。 */
#include "My_task.h"
#include "MotorApp.h"
#include <math.h>

volatile uint8_t g_upstairs_start = 0u;
volatile Upstairs_Debug_t g_upstairs_debug = {
    .move_2006_result = {1, 1}, .move_id5_result = 1,
    .second_stair = {.move_result = 1, .id5_move_result = 1}
};

/* 共用进度0~1：加速0.2s、匀速0.4s、减速0.2s，理论参考时长0.8s。
 * 进度速度5/3 (1/s)，进度加速度25/3 (1/s²)，为实际追位留约0.2s。 */
#define UPSTAIRS_PROGRESS_RATE  (5.0f / 3.0f)
#define UPSTAIRS_PROGRESS_ACCEL (25.0f / 3.0f)

typedef enum {
    frist_stairs = 0,
    second_stairs,
    third_stairs,
    fault_stairs
} up_state;

static void upstairs_debug_update(uint8_t state, uint8_t group,
                                  uint8_t issued, uint8_t started)
{
    uint8_t i;

    g_upstairs_debug.state = state;
    g_upstairs_debug.step = (uint8_t)(group + 1u);
    g_upstairs_debug.issued = issued;
    g_upstairs_debug.started = started;
    g_upstairs_debug.pair_2006 = *MotorApp_Get2006PairStatus();
    if (g_upstairs_debug.timing.status == 1u) {
        uint32_t timing_now = HAL_GetTick();
        g_upstairs_debug.timing.total_ms = timing_now - g_upstairs_debug.timing.step1_start_ms;
        if (group == 0u) {
            g_upstairs_debug.timing.step1_ms = g_upstairs_debug.timing.total_ms;
        } else if (group == 1u) {
            g_upstairs_debug.timing.step2_ms = timing_now - g_upstairs_debug.timing.step2_start_ms;
        } else {
            g_upstairs_debug.timing.elapsed_ms = timing_now - g_upstairs_debug.timing.step2_end_ms;
            if (group == 3u) {
                g_upstairs_debug.timing.step4_ms = timing_now - g_upstairs_debug.timing.step4_start_ms;
            } else {
                g_upstairs_debug.timing.step3_ms = g_upstairs_debug.timing.elapsed_ms;
            }
        }
        if (state == (uint8_t)fault_stairs) {
            g_upstairs_debug.timing.status = 3u;
        }
    }
    if (g_upstairs_debug.second_stair.status == 1u) {
        uint32_t timing_now = HAL_GetTick();
        g_upstairs_debug.second_stair.total_ms = timing_now - g_upstairs_debug.second_stair.step1_start_ms;
        if (group == 0u) {
            g_upstairs_debug.second_stair.step1_ms = g_upstairs_debug.second_stair.total_ms;
        } else if (group == 1u) {
            g_upstairs_debug.second_stair.step2_ms = timing_now - g_upstairs_debug.second_stair.step2_start_ms;
        } else if (group == 2u) {
            g_upstairs_debug.second_stair.step3_ms = timing_now - g_upstairs_debug.second_stair.step3_start_ms;
        }
        if (state == (uint8_t)fault_stairs) { g_upstairs_debug.second_stair.status = 3u; }
    }
    for (i = 0u; i < 5u; i++) {
        Axis_t *axis = MotorApp_GetAxis(i);
        g_upstairs_debug.axis[i].motor_id = axis->motor->id;
        g_upstairs_debug.axis[i].state = (uint8_t)axis->state;
        g_upstairs_debug.axis[i].planner_active = axis->traj.active;
        g_upstairs_debug.axis[i].command = axis->motor->cmd;
        g_upstairs_debug.axis[i].speed_rpm = axis->motor->raw_speed;
        g_upstairs_debug.axis[i].feedback_current = axis->motor->raw_current;
        g_upstairs_debug.axis[i].total_counts = axis->motor->total_counts;
        g_upstairs_debug.axis[i].error_turns =
            (axis->traj.target - axis->motor->angle_rad) * axis->motor->model->gear_ratio / 6.28318530718f;
        g_upstairs_debug.axis[i].settled_ms = axis->settled_s * 1000.0f;
        g_upstairs_debug.axis[i].rx_count = axis->motor->rx_count;
    }
    g_upstairs_debug.updates++;
}

/* 只记录第二步为何未完成；不修改电机、PID或切换条件。 */
static void upstairs_step2_record(uint32_t now, uint32_t delta)
{
    uint8_t i;
    for (i = 0u; i < 3u; i++) {
        Axis_t *a = MotorApp_GetAxis(i);
        volatile Upstairs_Step2Debug_t *d = &g_upstairs_debug.step2_axis[i];
        float error = a->traj.target - a->motor->angle_rad;
        float ratio = a->motor->model->gear_ratio;
        d->motor_id = a->motor->id;
        if (d->done_ms != 0u) { continue; }
        if (a->state == AXIS_IDLE) {
            d->done_ms = now - g_upstairs_debug.timing.step2_start_ms;
            continue;
        }
        d->last_error_turns = error * ratio / 6.28318530718f;
        d->last_speed_rpm = a->motor->raw_speed;
        d->last_command = a->motor->cmd;
        if (a->state == AXIS_FAULT) { d->last_reason = 5u; }
        else if (a->traj.active != 0u) { d->last_reason = 1u; d->planner_ms += delta; }
        else if (fabsf(error) > MOTORAPP_POSITION_TOLERANCE_ROTOR_TURNS * 6.28318530718f / ratio) {
            d->last_reason = 2u; d->position_ms += delta;
        } else if (fabsf(a->motor->speed_rad_s) > MOTORAPP_SPEED_TOLERANCE_ROTOR_RPM * 6.28318530718f / (60.0f * ratio)) {
            d->last_reason = 3u; d->speed_ms += delta;
        } else { d->last_reason = 4u; d->settle_ms += delta; }
    }
}

static uint8_t axis_done(uint8_t index)
{
    return (uint8_t)(MotorApp_GetAxis(index)->state == AXIS_IDLE);
}

void upstairs_task(void)
{
    uint32_t last, now;
    uint32_t step2_last_ms = 0u;
    float dt;
    up_state state = frist_stairs;
    uint8_t group = 0u;  /* 0/1=3508动作，2=2006与ID5联动，3=2006追加行程 */
    uint8_t issued = 0u;
    uint8_t started = 0u;
    uint8_t i;

    MotorApp_Init();
    last = osKernelGetTickCount();
    for (;;) {
        osDelay(1u);
        now = osKernelGetTickCount();
        dt = (float)(now - last) / (float)configTICK_RATE_HZ;
        last = now;
        if (dt < 0.0001f) { dt = 0.0001f; }

        /* 等待启动、运动中、到位后，都持续运行位置环和反馈更新。 */
        MotorApp_Update(dt);
        if (started == 0u) {
            if (g_upstairs_start != 1u) {
                upstairs_debug_update((uint8_t)state, group, issued, started);
                continue;
            }
            started = 1u;
            g_upstairs_debug.timing.step1_start_ms = HAL_GetTick();
            g_upstairs_debug.timing.status = 1u;
        }

        switch (state) {
        case frist_stairs: {
            /* 每一步圈数都相对该步起点，单位为减速前转子圈数。 */
            switch (group) {
            case 0u: {
                /* 第一步：3508 ID1、ID2 同步运动，ID5 保持原位。
                 * 数组仍按 APP 的 ID5 / ID2 / ID1 顺序填写。 */
                static const float s_first_stair_rotor_turns[3] = {
                    0.0f,         /* CAN3 ID5：本步不动 */
                    -7.6812f, /* CAN3 ID2：保留当前圈数 */ //-7.6812f刚好触碰
                    -34.430979f   /* CAN3 ID1：保留当前圈数 */
                };
                if (issued == 0u) {
                    if (MotorApp_Move3508TurnsSync(s_first_stair_rotor_turns,
                            UPSTAIRS_PROGRESS_RATE, UPSTAIRS_PROGRESS_ACCEL) == MOTORAPP_OK) {
                        issued = 1u;
                    }
                } else {
                    for (i = 0u; i < 3u; i++) {
                        if (MotorApp_GetAxis(i)->state == AXIS_FAULT) { state = fault_stairs; }
                    }
                    if ((state != fault_stairs) && (axis_done(0u) != 0u) &&
                        (axis_done(1u) != 0u) && (axis_done(2u) != 0u)) {
                        group = 1u;
                        issued = 0u;
                        g_upstairs_debug.timing.step2_start_ms = HAL_GetTick();
                        g_upstairs_debug.timing.step1_ms = g_upstairs_debug.timing.step2_start_ms -
                                                         g_upstairs_debug.timing.step1_start_ms;
                    }
                }
                break;
            }
            case 1u: {
                /* 第二步：3508 ID1、ID2、ID5 同步运动。
                 * 新行程未确定，先留 0；在这里分别填写本步圈数。 */
                static const float s_first_stair_step2_rotor_turns[3] = {
                    35.0f, /* CAN3 ID5：待填写 */
                    0.0f, /* CAN3 ID2：待填写 */  //-27.6711557f限位
                    -35.0f  /* CAN3 ID1：待填写 */
                };
                if (issued == 0u) {
                    if (MotorApp_Move3508TurnsSync(s_first_stair_step2_rotor_turns,
                            UPSTAIRS_PROGRESS_RATE, UPSTAIRS_PROGRESS_ACCEL) == MOTORAPP_OK) {
                        issued = 1u;
                        step2_last_ms = HAL_GetTick();
                    }
                } else {
                    uint32_t step2_now = HAL_GetTick();
                    upstairs_step2_record(step2_now, step2_now - step2_last_ms);
                    step2_last_ms = step2_now;
                    for (i = 0u; i < 3u; i++) {
                        if (MotorApp_GetAxis(i)->state == AXIS_FAULT) { state = fault_stairs; }
                    }
                    if ((state != fault_stairs) && (axis_done(0u) != 0u) &&
                        (axis_done(1u) != 0u) && (axis_done(2u) != 0u)) {
                        group = 2u;
                        issued = 0u;
                        g_upstairs_debug.timing.step2_end_ms = HAL_GetTick();
                        g_upstairs_debug.timing.step2_ms = g_upstairs_debug.timing.step2_end_ms -
                                                         g_upstairs_debug.timing.step2_start_ms;
                    }
                }
                break;
            }
            case 2u: {
                /* 第三步：双2006共同驱动；正圈数 = ID3负转、ID4正转。
                 * 相对本步起点，统一填写减速前转子圈数。 */
                static const float s_first_stair_2006_rotor_turns = 50.19518086f;
                static const float s_halfway_id5_rotor_turns = -35.0f;
                const MotorApp_2006PairStatus_t *pair = MotorApp_Get2006PairStatus();
                Axis_t *lift = MotorApp_GetAxis(0u);
                if (issued != 0u) {
                    g_upstairs_debug.pair_timing[0] = pair->timing;
                    if (pair->timing.status == 2u) {
                        g_upstairs_debug.timing.step3_after_pair_ms = HAL_GetTick() -
                            pair->timing.start_ms - pair->timing.total_ms;
                    }
                }
                if (issued == 0u) {
                    int result = MotorApp_Move2006Pair(s_first_stair_2006_rotor_turns);
                    g_upstairs_debug.move_2006_result[0] = result;
                    g_upstairs_debug.move_2006_result[1] = result;
                    if (result == MOTORAPP_OK) {
                        issued = 3u; /* 兼容原Watch值：两台作为一组接受 */
                        g_upstairs_debug.pair_timing[0] = pair->timing;
                    } else if ((result == MOTORAPP_ERR_PARAM) || (result == MOTORAPP_ERR_FAULT)) {
                        state = fault_stairs;
                    }
                } else if (pair->state == MOTORAPP_PAIR_FAULT) {
                    state = fault_stairs;
                } else {
                    float halfway = s_first_stair_2006_rotor_turns * 0.75f;
                    uint8_t both_halfway = 0u;
                    /* 两侧实际行程均达到75%即启动ID5，与2006并行；不等待组到位。 */
                    if (s_first_stair_2006_rotor_turns > 0.0f) {
                        both_halfway = (uint8_t)((pair->actual_turns[0] >= halfway) &&
                                                 (pair->actual_turns[1] >= halfway));
                    } else if (s_first_stair_2006_rotor_turns < 0.0f) {
                        both_halfway = (uint8_t)((pair->actual_turns[0] <= halfway) &&
                                                 (pair->actual_turns[1] <= halfway));
                    }
                    /* 正常75%触发；若2006一侧先到位，则在结束本段时补发ID5。 */
                    if (((issued & 4u) == 0u) && (s_first_stair_2006_rotor_turns != 0.0f) &&
                        ((both_halfway != 0u) || (pair->state == MOTORAPP_PAIR_DONE))) {
                        int result = MotorApp_MoveTurns(lift, s_halfway_id5_rotor_turns);
                        g_upstairs_debug.move_id5_result = result;
                        if (result == MOTORAPP_OK) {
                            issued = (uint8_t)(issued | 4u);
                            g_upstairs_debug.id5_half_started = 1u;
                        } else if ((result == MOTORAPP_ERR_PARAM) || (result == MOTORAPP_ERR_FAULT)) {
                            state = fault_stairs;
                        }
                    }
                    if (((issued & 4u) != 0u) && (lift->state == AXIS_FAULT)) {
                        state = fault_stairs;
                    }
                    /* ID5请求已接受即可进入下一步，不等待ID5完成。 */
                    if ((state != fault_stairs) && (pair->state == MOTORAPP_PAIR_DONE) &&
                        ((s_first_stair_2006_rotor_turns == 0.0f) ||
                         ((issued & 4u) != 0u))) {
                        group = 3u;
                        issued = 0u;
                        g_upstairs_debug.timing.step4_start_ms = HAL_GetTick();
                        g_upstairs_debug.timing.step3_ms = g_upstairs_debug.timing.step4_start_ms -
                                                         g_upstairs_debug.timing.step2_end_ms;
                    }
                }
                break;
            }
            case 3u: {
                /* 双2006从此刻的位置再走配置的追加圈数，方向与上一段相同。
                 * ID5继续独立完成并保持；本段任一侧到位即结束第一级流程。 */
                static const float s_first_stair_2006_extra_rotor_turns = 42.0f;
                const MotorApp_2006PairStatus_t *pair = MotorApp_Get2006PairStatus();
                if (issued != 0u) { g_upstairs_debug.pair_timing[1] = pair->timing; }
                if (issued == 0u) {
                    int result = MotorApp_Move2006Pair(s_first_stair_2006_extra_rotor_turns);
                    g_upstairs_debug.move_2006_result[0] = result;
                    g_upstairs_debug.move_2006_result[1] = result;
                    if (result == MOTORAPP_OK) {
                        issued = 3u;
                        g_upstairs_debug.pair_timing[1] = pair->timing;
                    } else if ((result == MOTORAPP_ERR_PARAM) || (result == MOTORAPP_ERR_FAULT)) {
                        state = fault_stairs;
                    }
                } else if (pair->state == MOTORAPP_PAIR_FAULT) {
                    state = fault_stairs;
                } else if (pair->state == MOTORAPP_PAIR_DONE) {
                    g_upstairs_debug.timing.step4_end_ms = HAL_GetTick();
                    g_upstairs_debug.timing.step4_ms = g_upstairs_debug.timing.step4_end_ms -
                                                     g_upstairs_debug.timing.step4_start_ms;
                    g_upstairs_debug.timing.elapsed_ms = g_upstairs_debug.timing.step4_end_ms -
                                                       g_upstairs_debug.timing.step2_end_ms;
                    g_upstairs_debug.timing.total_ms = g_upstairs_debug.timing.step4_end_ms -
                                                     g_upstairs_debug.timing.step1_start_ms;
                    g_upstairs_debug.timing.status = 2u;
                    g_upstairs_debug.first_stair_done = 1u;
                    g_upstairs_debug.second_stair.step1_start_ms = g_upstairs_debug.timing.step4_end_ms;
                    g_upstairs_debug.second_stair.status = 1u;
                    state = second_stairs;
                    group = 0u;
                    issued = 0u;
                }
                break;
            }
            default:
                break;
            }
            break;
        }
        case second_stairs: {
            switch (group) {
            case 0u: {
                /* 第二级第一步，相对本步真实起点，单位为减速前转子圈数。 */
                static const float s_second_stair_step1_rotor_turns[3] = {
                    35.0f,  /* CAN3 ID5 */
                    0.0f,   /* CAN3 ID2：保持 */
                    -35.0f  /* CAN3 ID1 */
                };
                if (issued == 0u) {
                    int result = MotorApp_Move3508TurnsSync(s_second_stair_step1_rotor_turns,
                            UPSTAIRS_PROGRESS_RATE, UPSTAIRS_PROGRESS_ACCEL);
                    g_upstairs_debug.second_stair.move_result = result;
                    if (result == MOTORAPP_OK) { issued = 1u; }
                } else {
                    for (i = 0u; i < 3u; i++) {
                        if (MotorApp_GetAxis(i)->state == AXIS_FAULT) { state = fault_stairs; }
                    }
                    if ((state != fault_stairs) && (axis_done(0u) != 0u) &&
                        (axis_done(1u) != 0u) && (axis_done(2u) != 0u)) {
                        g_upstairs_debug.second_stair.step2_start_ms = HAL_GetTick();
                        g_upstairs_debug.second_stair.step1_ms = g_upstairs_debug.second_stair.step2_start_ms -
                            g_upstairs_debug.second_stair.step1_start_ms;
                        group = 1u;
                        issued = 0u;
                    }
                }
                break;
            }
            case 1u: {
                /* 第二级第二步：双2006走首段，75%时并行启动ID5。 */
                static const float s_second_stair_2006_rotor_turns = 50.19518086f;
                static const float s_second_stair_id5_rotor_turns = -35.0f;
                const MotorApp_2006PairStatus_t *pair = MotorApp_Get2006PairStatus();
                Axis_t *lift = MotorApp_GetAxis(0u);
                if (issued != 0u) { g_upstairs_debug.second_stair.pair_timing[0] = pair->timing; }
                if (issued == 0u) {
                    int result = MotorApp_Move2006Pair(s_second_stair_2006_rotor_turns);
                    g_upstairs_debug.second_stair.move_result = result;
                    if (result == MOTORAPP_OK) {
                        issued = 3u;
                        g_upstairs_debug.second_stair.pair_timing[0] = pair->timing;
                    } else if ((result == MOTORAPP_ERR_PARAM) || (result == MOTORAPP_ERR_FAULT)) {
                        state = fault_stairs;
                    }
                } else if (pair->state == MOTORAPP_PAIR_FAULT) {
                    state = fault_stairs;
                } else {
                    float threshold = s_second_stair_2006_rotor_turns * 0.75f;
                    uint8_t both_75 = 0u;
                    if (s_second_stair_2006_rotor_turns > 0.0f) {
                        both_75 = (uint8_t)((pair->actual_turns[0] >= threshold) &&
                                           (pair->actual_turns[1] >= threshold));
                    } else if (s_second_stair_2006_rotor_turns < 0.0f) {
                        both_75 = (uint8_t)((pair->actual_turns[0] <= threshold) &&
                                           (pair->actual_turns[1] <= threshold));
                    }
                    /* 一侧先到位时补发ID5；请求成功后不再重复下发。 */
                    if (((issued & 4u) == 0u) && (s_second_stair_2006_rotor_turns != 0.0f) &&
                        ((both_75 != 0u) || (pair->state == MOTORAPP_PAIR_DONE))) {
                        int result = MotorApp_MoveTurns(lift, s_second_stair_id5_rotor_turns);
                        g_upstairs_debug.second_stair.id5_move_result = result;
                        if (result == MOTORAPP_OK) {
                            issued = (uint8_t)(issued | 4u);
                            g_upstairs_debug.second_stair.id5_started = 1u;
                        } else if ((result == MOTORAPP_ERR_PARAM) || (result == MOTORAPP_ERR_FAULT)) {
                            state = fault_stairs;
                        }
                    }
                    if (((issued & 4u) != 0u) && (lift->state == AXIS_FAULT)) { state = fault_stairs; }
                    if ((state != fault_stairs) && (pair->state == MOTORAPP_PAIR_DONE) &&
                        ((s_second_stair_2006_rotor_turns == 0.0f) || ((issued & 4u) != 0u))) {
                        g_upstairs_debug.second_stair.step3_start_ms = HAL_GetTick();
                        g_upstairs_debug.second_stair.step2_ms = g_upstairs_debug.second_stair.step3_start_ms -
                            g_upstairs_debug.second_stair.step2_start_ms;
                        group = 2u;
                        issued = 0u;
                    }
                }
                break;
            }
            case 2u: {
                /* 第二级第三步：相对当前真实位置再走42转子圈，ID5独立运行。 */
                static const float s_second_stair_2006_extra_rotor_turns = 42.0f;
                const MotorApp_2006PairStatus_t *pair = MotorApp_Get2006PairStatus();
                if (issued != 0u) { g_upstairs_debug.second_stair.pair_timing[1] = pair->timing; }
                if (issued == 0u) {
                    int result = MotorApp_Move2006Pair(s_second_stair_2006_extra_rotor_turns);
                    g_upstairs_debug.second_stair.move_result = result;
                    if (result == MOTORAPP_OK) {
                        issued = 3u;
                        g_upstairs_debug.second_stair.pair_timing[1] = pair->timing;
                    } else if ((result == MOTORAPP_ERR_PARAM) || (result == MOTORAPP_ERR_FAULT)) {
                        state = fault_stairs;
                    }
                } else if (pair->state == MOTORAPP_PAIR_FAULT) {
                    state = fault_stairs;
                } else if (pair->state == MOTORAPP_PAIR_DONE) {
                    uint32_t timing_now = HAL_GetTick();
                    g_upstairs_debug.second_stair.step3_ms = timing_now - g_upstairs_debug.second_stair.step3_start_ms;
                    g_upstairs_debug.second_stair.total_ms = timing_now - g_upstairs_debug.second_stair.step1_start_ms;
                    g_upstairs_debug.second_stair.status = 2u;
                    state = third_stairs;
                    group = 0u;
                    issued = 0u;
                }
                break;
            }
            default:
                break;
            }
            break;
        }
        default:
            /* 后续动作尚未实现，或发生故障：不再下发新行程，继续保持。 */
            break;
        }
        upstairs_debug_update((uint8_t)state, group, issued, started);
    }
}
