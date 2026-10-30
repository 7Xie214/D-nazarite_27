#include "ChassisTask.h"
#include "ChassisApp.h"
#include "cmsis_os2.h"

#if CHASSIS_ENABLE_SWERVE
extern osThreadId_t Chassis_TaskHandle;
extern osThreadId_t Upstairs_TaskHandle;
#endif

osStatus_t ChassisTask_RequestFrontMove(void)
{
#if CHASSIS_ENABLE_SWERVE
    uint32_t flags;
    flags = osThreadFlagsClear(CHASSIS_TASK_DONE_FLAG | CHASSIS_TASK_ERROR_FLAG);
    if ((flags & osFlagsError) != 0u) {
        return osErrorResource;
    }
    flags = osThreadFlagsSet(Chassis_TaskHandle, CHASSIS_TASK_START_FLAG);
    return ((flags & osFlagsError) != 0u) ? osErrorResource : osOK;
#else
    return osErrorResource;
#endif
}

uint32_t ChassisTask_PollReply(void)
{
#if CHASSIS_ENABLE_SWERVE
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

void ChassisTask(void *argument)
{
#if CHASSIS_ENABLE_SWERVE
    uint32_t tick_hz;
    uint32_t period_ticks;
    uint32_t wake_tick;
    uint32_t last_tick;
    uint32_t now;
    uint32_t flags;
    uint32_t pending = 0u;
    uint32_t active = 0u;
    uint32_t reply_pending = 0u;
    int32_t scheduler_state;
    ChassisFront_StartResult_t start_result;
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
        flags = osThreadFlagsWait(CHASSIS_TASK_START_FLAG, osFlagsWaitAny, 0u);
        /* ErrorResource also contains bit 0: reject errors before flag bits. */
        if (((flags & osFlagsError) == 0u) &&
            ((flags & CHASSIS_TASK_START_FLAG) != 0u) &&
            (pending == 0u) && (active == 0u) && (reply_pending == 0u)) {
            pending = 1u;
        }

        ChassisApp_UpdateFrontMove(dt);
        if (pending != 0u) {
            /* Refresh feedback/holds first, then capture each relative target. */
            if ((g_chassis_app_status != CHASSIS_APP_READY) &&
                (g_chassis_app_status != CHASSIS_APP_FAULT_FEEDBACK)) {
                /* Only missing feedback is a recoverable readiness wait. */
                start_result = CHASSIS_FRONT_START_BAD_CONFIG;
            } else {
                start_result = ChassisApp_MoveFrontTurns(
                    CHASSIS_FRONT_LEFT_MOVE_ROTOR_TURNS,
                    CHASSIS_FRONT_RIGHT_MOVE_ROTOR_TURNS);
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
        if (reply_pending != 0u) {
            /* Upstairs must not run between draining START and replying. */
            scheduler_state = osKernelLock();
            if (scheduler_state >= 0) {
                flags = osThreadFlagsClear(CHASSIS_TASK_START_FLAG);
                if ((flags & osFlagsError) == 0u) {
                    flags = osThreadFlagsSet(Upstairs_TaskHandle, reply_pending);
                    if ((flags & osFlagsError) == 0u) {
                        reply_pending = 0u;
                    }
                }
                /* This CMSIS port adds nesting in RestoreLock(1), whereas
                 * Lock() leaves an existing suspension unchanged. */
                if (scheduler_state == 0) {
                    (void)osKernelRestoreLock(scheduler_state);
                }
            }
        }
    }
#else
    (void)argument;
    osThreadExit();
#endif /* CHASSIS_ENABLE_SWERVE */
}
