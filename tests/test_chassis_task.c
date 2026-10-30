#include "ChassisTask.h"
#include "ChassisApp.h"
#include "cmsis_os2.h"
#include <math.h>
#include <setjmp.h>
#include <stdio.h>

static jmp_buf task_exit;
static uint32_t now_tick, tick_hz, overrun, jitter;
static unsigned int checks, failures, init_calls, update_calls, delays, exits;
static float actual_dt[3];
osThreadId_t Chassis_TaskHandle = (osThreadId_t)1;
osThreadId_t Upstairs_TaskHandle = (osThreadId_t)2;
ChassisFront_State_t g_chassis_front_state;
ChassisApp_Status_t g_chassis_app_status;

#define CHECK(condition) do { checks++; if (!(condition)) { \
    printf("FAIL line %d: %s\n", __LINE__, #condition); failures++; } } while (0)

uint32_t osKernelGetTickFreq(void) { return tick_hz; }
uint32_t osKernelGetTickCount(void) { return now_tick; }
osStatus_t osDelayUntil(uint32_t deadline)
{
    uint32_t wait = deadline - now_tick;
    CHECK(wait > 0u && wait < 0x80000000u);
    now_tick = deadline;
    if (delays == 0u) { now_tick += jitter; }
    delays++;
    return osOK;
}
void osThreadExit(void) { exits++; longjmp(task_exit, 1); }
int32_t osKernelLock(void) { return 0; }
int32_t osKernelRestoreLock(int32_t old) { return old; }
void ChassisApp_Init(void) { init_calls++; }
void ChassisApp_UpdateFrontMove(float dt)
{
    actual_dt[update_calls] = dt;
    update_calls++;
    if (update_calls == 1u) { now_tick += overrun; }
    if (update_calls == 3u) { longjmp(task_exit, 1); }
}
ChassisFront_StartResult_t ChassisApp_MoveFrontTurns(float left, float right)
{
    (void)left; (void)right;
    return CHASSIS_FRONT_START_NOT_READY;
}
uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout)
{
    (void)flags; (void)options;
    CHECK(timeout == 0u);
    return osFlagsErrorResource;
}
uint32_t osThreadFlagsClear(uint32_t flags) { (void)flags; return 0u; }
uint32_t osThreadFlagsSet(osThreadId_t thread, uint32_t flags)
{
    (void)thread; return flags;
}

#if CHASSIS_ENABLE_SWERVE
static void run_case(uint32_t start, uint32_t hz, uint32_t busy, uint32_t late)
{
    uint32_t period = (hz + 999u) / 1000u;
    init_calls = 0u; update_calls = 0u; delays = 0u;
    now_tick = start; tick_hz = hz; overrun = busy; jitter = late;
    if (setjmp(task_exit) == 0) { ChassisTask(NULL); }
    CHECK(init_calls == 1u);
    CHECK(update_calls == 3u);
    CHECK(delays == 3u);
    CHECK(fabsf(actual_dt[0] - (float)(period + late) / (float)hz) < 0.000001f);
    CHECK(fabsf(actual_dt[1] - (float)(period + busy) / (float)hz) < 0.000001f);
    CHECK(fabsf(actual_dt[2] - (float)period / (float)hz) < 0.000001f);
}
#endif

int main(void)
{
#if CHASSIS_ENABLE_SWERVE
    run_case(10u, 1000u, 0u, 0u);
    run_case(UINT32_MAX - 1u, 1000u, 0u, 0u);
    run_case(10u, 1000u, 3u, 0u);
    run_case(10u, 1000u, 0u, 2u);
    run_case(10u, 500u, 0u, 0u);
    run_case(10u, 2000u, 0u, 0u);
    run_case(10u, 1024u, 0u, 0u);
#else
    if (setjmp(task_exit) == 0) { ChassisTask(NULL); }
    CHECK(exits == 1u);
    CHECK(init_calls == 0u);
    CHECK(update_calls == 0u);
    CHECK(delays == 0u);
#endif
    printf("%u chassis task checks, %u failures\n", checks, failures);
    return (failures == 0u) ? 0 : 1;
}
