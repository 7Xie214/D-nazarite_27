#include "ChassisTask.h"
#include "ChassisApp.h"
#include "cmsis_os2.h"

#if CHASSIS_ENABLE_SWERVE && CHASSIS_TASK_FRONT_MOVE_ENABLE
extern osThreadId_t Chassis_TaskHandle;
extern osThreadId_t Upstairs_TaskHandle;
static volatile float s_front_request_turns[2] = {
    CHASSIS_FRONT_LEFT_MOVE_ROTOR_TURNS, CHASSIS_FRONT_RIGHT_MOVE_ROTOR_TURNS
};
static volatile uint8_t s_front_request_busy;
static float s_front_actual_turns[2];
static uint32_t s_front_progress_sequence;
static uint8_t s_front_progress_paused;
static volatile float s_front_request_pause;
static volatile uint32_t s_front_resume_sequence;
#endif

osStatus_t ChassisTask_RequestFrontMove(void)
{
#if CHASSIS_ENABLE_SWERVE && CHASSIS_TASK_FRONT_MOVE_ENABLE
    return ChassisTask_RequestFrontMoveTurns(CHASSIS_FRONT_LEFT_MOVE_ROTOR_TURNS,
                                            CHASSIS_FRONT_RIGHT_MOVE_ROTOR_TURNS);
#else
    return osErrorResource;
#endif
}

osStatus_t ChassisTask_RequestFrontMoveTurns(float left_rotor_turns, float right_rotor_turns)
{
    return ChassisTask_RequestFrontMoveWithPause(left_rotor_turns, right_rotor_turns, 0.0f);
}

osStatus_t ChassisTask_RequestFrontMoveWithPause(float left_rotor_turns, float right_rotor_turns,
                                               float pause_fraction)
{
#if CHASSIS_ENABLE_SWERVE && CHASSIS_TASK_FRONT_MOVE_ENABLE
    int32_t scheduler_state;
    osStatus_t result = osErrorResource;
    uint32_t flags;
    scheduler_state = osKernelLock();
    if (scheduler_state < 0) { return osErrorResource; }
    if (s_front_request_busy == 0u) {
        flags = osThreadFlagsClear(CHASSIS_TASK_DONE_FLAG | CHASSIS_TASK_ERROR_FLAG);
        if ((flags & osFlagsError) == 0u) {
            /* The chassis task cannot run between publishing the pair and START. */
            s_front_request_turns[0] = left_rotor_turns;
            s_front_request_turns[1] = right_rotor_turns;
            s_front_request_pause = pause_fraction;
            s_front_request_busy = 1u;
            flags = osThreadFlagsSet(Chassis_TaskHandle, CHASSIS_TASK_START_FLAG);
            if ((flags & osFlagsError) == 0u) { result = osOK; }
            else { s_front_request_busy = 0u; }
        }
    }
    if (scheduler_state == 0) { (void)osKernelRestoreLock(scheduler_state); }
    return result;
#else
    (void)left_rotor_turns; (void)right_rotor_turns; (void)pause_fraction;
    return osErrorResource;
#endif
}

osStatus_t ChassisTask_ResumeFrontMove(uint32_t move_sequence)
{
    osStatus_t result = osErrorResource;
#if CHASSIS_ENABLE_SWERVE && CHASSIS_TASK_FRONT_MOVE_ENABLE
    int32_t scheduler_state = osKernelLock();
    if (scheduler_state >= 0) {
        if (s_front_request_busy && s_front_progress_paused && s_front_progress_sequence == move_sequence) {
            s_front_resume_sequence = move_sequence;
            if ((osThreadFlagsSet(Chassis_TaskHandle, CHASSIS_TASK_RESUME_FLAG) & osFlagsError) == 0u) { result = osOK; }
        }
        if (scheduler_state == 0) { (void)osKernelRestoreLock(scheduler_state); }
    }
#else
    (void)move_sequence;
#endif
    return result;
}

uint32_t ChassisTask_PollReply(void)
{
#if CHASSIS_ENABLE_SWERVE && CHASSIS_TASK_FRONT_MOVE_ENABLE
    uint32_t flags;
    flags = osThreadFlagsWait(CHASSIS_TASK_DONE_FLAG | CHASSIS_TASK_ERROR_FLAG,
                              osFlagsWaitAny, 0u);
    if ((flags & osFlagsError) != 0u) {
        return 0u;
    }
    return flags & (CHASSIS_TASK_DONE_FLAG | CHASSIS_TASK_ERROR_FLAG);
#else
    return 0u;
#endif
}

uint32_t ChassisTask_GetFrontProgress(float actual_rotor_turns[2])
{
    uint32_t sequence = 0u;
    actual_rotor_turns[0] = 0.0f; actual_rotor_turns[1] = 0.0f;
#if CHASSIS_ENABLE_SWERVE && CHASSIS_TASK_FRONT_MOVE_ENABLE
    int32_t scheduler_state = osKernelLock();
    if (scheduler_state >= 0) {
        actual_rotor_turns[0] = s_front_actual_turns[0];
        actual_rotor_turns[1] = s_front_actual_turns[1];
        sequence = s_front_progress_sequence;
        if (scheduler_state == 0) { (void)osKernelRestoreLock(scheduler_state); }
    }
#endif
    return sequence;
}

void ChassisTask(void *argument)
{
#if CHASSIS_ENABLE_SWERVE
    uint32_t tick_hz;
    uint32_t period_ticks;
    uint32_t wake_tick;
    uint32_t last_tick;
    uint32_t now;
#if CHASSIS_TASK_FRONT_MOVE_ENABLE
    uint32_t flags;
    uint32_t pending = 0u;
    uint32_t active = 0u;
    uint32_t reply_pending = 0u;
    int32_t scheduler_state;
    ChassisFront_StartResult_t start_result;
    float pending_left_turns = CHASSIS_FRONT_LEFT_MOVE_ROTOR_TURNS;
    float pending_right_turns = CHASSIS_FRONT_RIGHT_MOVE_ROTOR_TURNS;
    float pending_pause = 0.0f;
    uint8_t resume_pending = 0u;
    uint32_t resume_sequence = 0u;
#endif
    float dt;

    (void)argument;
    ChassisApp_Init();

    tick_hz = osKernelGetTickFreq();
    /* Round 1 ms up to an available kernel tick, with at least one tick. */
    period_ticks = tick_hz / 1000u;
    if ((tick_hz % 1000u) != 0u) {
        period_ticks++;
    }
    if (period_ticks == 0u) {
        period_ticks = 1u;
    }
    last_tick = osKernelGetTickCount();
    wake_tick = last_tick;

    for (;;) {
        now = osKernelGetTickCount();
        /* Rebase after an overrun instead of executing catch-up iterations.
         * Unsigned differences also handle the kernel tick counter wrapping. */
        if ((uint32_t)(now - wake_tick) >= period_ticks) {
            wake_tick = now;
        }
        wake_tick += period_ticks;
        (void)osDelayUntil(wake_tick);

        now = osKernelGetTickCount();
        dt = (float)(uint32_t)(now - last_tick) / (float)tick_hz;
        last_tick = now;
#if CHASSIS_TASK_FRONT_MOVE_ENABLE
        flags = osThreadFlagsWait(CHASSIS_TASK_START_FLAG | CHASSIS_TASK_RESUME_FLAG, osFlagsWaitAny, 0u);
        /* ErrorResource also contains bit 0: reject errors before flag bits. */
        if (((flags & osFlagsError) == 0u) &&
            ((flags & CHASSIS_TASK_START_FLAG) != 0u) &&
            (pending == 0u) && (active == 0u) && (reply_pending == 0u)) {
            s_front_request_busy = 1u;
            pending_left_turns = s_front_request_turns[0];
            pending_right_turns = s_front_request_turns[1];
            pending_pause = s_front_request_pause;
            resume_pending = 0u;
            pending = 1u;
        }
        if (((flags & osFlagsError) == 0u) && (flags & CHASSIS_TASK_RESUME_FLAG)) {
            resume_sequence = s_front_resume_sequence;
            resume_pending = 1u;
        }

        ChassisApp_UpdateFrontPair(dt);
        if (pending != 0u) {
            /* Refresh feedback/holds first, then capture each relative target. */
            if ((g_chassis_app_status != CHASSIS_APP_READY) &&
                (g_chassis_app_status != CHASSIS_APP_FAULT_FEEDBACK)) {
                /* Only missing feedback is a recoverable readiness wait. */
                start_result = CHASSIS_FRONT_START_BAD_CONFIG;
            } else {
                start_result = ChassisApp_MoveFrontPairTurnsWithPause(
                    pending_left_turns, pending_right_turns, pending_pause);
            }
            if (start_result == CHASSIS_FRONT_START_OK) {
                pending = 0u;
                active = 1u;
            } else if (start_result != CHASSIS_FRONT_START_NOT_READY) {
                pending = 0u;
                reply_pending = CHASSIS_TASK_ERROR_FLAG;
            }
        }
        if (active != 0u) {
            if (g_chassis_front_state == CHASSIS_FRONT_DONE) {
                active = 0u;
                reply_pending = CHASSIS_TASK_DONE_FLAG;
            } else if (g_chassis_front_state == CHASSIS_FRONT_FAULT) {
                active = 0u;
                reply_pending = CHASSIS_TASK_ERROR_FLAG;
            }
        }
        if (active && resume_pending && g_chassis_front_pair.move_sequence == resume_sequence) {
            if (ChassisApp_ResumeFrontPair()) { resume_pending = 0u; }
        }
        /* APP may be preempted between wheel updates. Publish only its complete
         * sample, together with the accepted-move number, before replying. */
        scheduler_state = osKernelLock();
        if (scheduler_state >= 0) {
            s_front_actual_turns[0] = g_chassis_front_pair.actual_turns[0];
            s_front_actual_turns[1] = g_chassis_front_pair.actual_turns[1];
            s_front_progress_sequence = g_chassis_front_pair.move_sequence;
            s_front_progress_paused = g_chassis_front_pair.paused;
            if (scheduler_state == 0) { (void)osKernelRestoreLock(scheduler_state); }
        }
        if (reply_pending != 0u) {
            /* Upstairs must not run between draining START and replying. */
            scheduler_state = osKernelLock();
            if (scheduler_state >= 0) {
                flags = osThreadFlagsClear(CHASSIS_TASK_START_FLAG | CHASSIS_TASK_RESUME_FLAG);
                if ((flags & osFlagsError) == 0u) {
                    flags = osThreadFlagsSet(Upstairs_TaskHandle, reply_pending);
                    if ((flags & osFlagsError) == 0u) {
                        reply_pending = 0u;
                        s_front_request_busy = 0u;
                    }
                }
                /* This CMSIS port adds nesting in RestoreLock(1), whereas
                 * Lock() leaves an existing suspension unchanged. */
                if (scheduler_state == 0) {
                    (void)osKernelRestoreLock(scheduler_state);
                }
            }
        }
#else
        /* 上电即回到已标定机械零位，不等待上层任务的启动通知。 */
        ChassisApp_UpdateSteerHold(dt);
#endif
    }
#else
    (void)argument;
    osThreadExit();
#endif /* CHASSIS_ENABLE_SWERVE */
}
