#include "ChassisTask.h"
#include "ChassisApp.h"
#include "cmsis_os2.h"
#include <math.h>
#include <setjmp.h>
#include <stdio.h>

osThreadId_t Chassis_TaskHandle = (osThreadId_t)1;
osThreadId_t Upstairs_TaskHandle = (osThreadId_t)2;
ChassisFront_State_t g_chassis_front_state;
ChassisApp_Status_t g_chassis_app_status;
static jmp_buf task_exit;
static uint32_t tick, chassis_flags, upstairs_flags;
static osThreadId_t self;
static unsigned int checks, failures, updates, accepted, attempts;
static unsigned int reply_attempts, replies, finish_cycle, ready_cycle;
static int bad_config, fail_move, fail_reply_once, fail_request;
static int before_start_fault, late_duplicate;
static int scheduler_locked, publication_gap_duplicate;
static unsigned int blocked_gap_attempts;

#define CHECK(condition) do { checks++; if (!(condition)) { \
    printf("FAIL line %d: %s\n", __LINE__, #condition); failures++; } } while (0)

uint32_t osKernelGetTickFreq(void) { return 1000u; }
uint32_t osKernelGetTickCount(void) { return tick; }
osStatus_t osDelayUntil(uint32_t deadline)
{
    CHECK(deadline - tick == 1u);
    tick = deadline;
    return osOK;
}
void osThreadExit(void) { longjmp(task_exit, 1); }
int32_t osKernelLock(void)
{
    int32_t old = scheduler_locked;
    scheduler_locked = 1;
    return old;
}
int32_t osKernelRestoreLock(int32_t old)
{
    scheduler_locked = (int)old;
    return old;
}
uint32_t osThreadFlagsWait(uint32_t flags, uint32_t options, uint32_t timeout)
{
    uint32_t *stored = (self == Chassis_TaskHandle) ? &chassis_flags : &upstairs_flags;
    uint32_t found = *stored & flags;
    CHECK(timeout == 0u);
    CHECK(options == osFlagsWaitAny);
    if (found == 0u) { return osFlagsErrorResource; }
    *stored &= ~found;
    return found;
}
uint32_t osThreadFlagsClear(uint32_t flags)
{
    uint32_t *stored = (self == Chassis_TaskHandle) ? &chassis_flags : &upstairs_flags;
    uint32_t old = *stored;
    *stored &= ~flags;
    return old;
}
uint32_t osThreadFlagsSet(osThreadId_t thread, uint32_t flags)
{
    if (thread == Chassis_TaskHandle) {
        if (fail_request) { return osFlagsErrorResource; }
        chassis_flags |= flags;
        return chassis_flags;
    }
    CHECK(thread == Upstairs_TaskHandle);
    if (publication_gap_duplicate) {
        /* An Upstairs time slice between START drain and reply publication. */
        if (!scheduler_locked) { chassis_flags |= CHASSIS_TASK_START_FLAG; }
        else { blocked_gap_attempts++; }
    }
    reply_attempts++;
    if (fail_reply_once && reply_attempts == 1u) { return osFlagsErrorResource; }
    replies++;
    upstairs_flags |= flags;
    return upstairs_flags;
}
void ChassisApp_Init(void) { g_chassis_front_state = CHASSIS_FRONT_HOLD; }
ChassisFront_StartResult_t ChassisApp_MoveFrontTurns(float left, float right)
{
    attempts++;
    CHECK(fabsf(left - 0.75f) < 0.00001f);
    CHECK(fabsf(right + 1.25f) < 0.00001f);
    if (bad_config) { return CHASSIS_FRONT_START_BAD_CONFIG; }
    if (before_start_fault) { return CHASSIS_FRONT_START_NOT_READY; }
    if (updates < ready_cycle) { return CHASSIS_FRONT_START_NOT_READY; }
    accepted++;
    g_chassis_front_state = CHASSIS_FRONT_MOVING;
    return CHASSIS_FRONT_START_OK;
}
void ChassisApp_UpdateFrontMove(float dt)
{
    CHECK(fabsf(dt - 0.001f) < 0.000001f);
    updates++;
    g_chassis_app_status = before_start_fault ? CHASSIS_APP_FAULT_CONFIG : CHASSIS_APP_READY;
    if (updates == 2u && accepted != 0u) { chassis_flags |= CHASSIS_TASK_START_FLAG; }
    if (updates == finish_cycle && g_chassis_front_state == CHASSIS_FRONT_MOVING) {
        if (late_duplicate) { chassis_flags |= CHASSIS_TASK_START_FLAG; }
        g_chassis_front_state = fail_move ? CHASSIS_FRONT_FAULT : CHASSIS_FRONT_DONE;
    }
    if (updates == 10u) { longjmp(task_exit, 1); }
}

static void run_handshake(int invalid, int fault, int retry, unsigned int ready)
{
    tick = 0u; updates = 0u; accepted = 0u; attempts = 0u;
    reply_attempts = 0u; replies = 0u; upstairs_flags = 0u;
    bad_config = invalid; fail_move = fault; fail_reply_once = retry;
    ready_cycle = ready; finish_cycle = 5u;
    chassis_flags = CHASSIS_TASK_START_FLAG;
    self = Chassis_TaskHandle;
    if (setjmp(task_exit) == 0) { ChassisTask(NULL); }
    CHECK(replies == 1u);
    CHECK(reply_attempts == (retry ? 2u : 1u));
    CHECK(accepted == ((invalid || before_start_fault) ? 0u : 1u));
    CHECK(upstairs_flags == ((invalid || fault || before_start_fault) ? CHASSIS_TASK_ERROR_FLAG : CHASSIS_TASK_DONE_FLAG));
    CHECK(scheduler_locked == 0);
    if (!invalid && !before_start_fault) { CHECK(attempts >= 1u); }
}

int main(void)
{
    self = Upstairs_TaskHandle;
    upstairs_flags = CHASSIS_TASK_DONE_FLAG | CHASSIS_TASK_ERROR_FLAG;
    CHECK(ChassisTask_RequestFrontMove() == osOK);
    CHECK(upstairs_flags == 0u);
    CHECK(chassis_flags == CHASSIS_TASK_START_FLAG);
    upstairs_flags = CHASSIS_TASK_DONE_FLAG;
    CHECK(ChassisTask_PollReply() == CHASSIS_TASK_DONE_FLAG);
    CHECK(ChassisTask_PollReply() == 0u); /* ErrorResource bits are not real flags. */
    fail_request = 1;
    CHECK(ChassisTask_RequestFrontMove() != osOK);
    fail_request = 0;
    run_handshake(0, 0, 0, 1u);
    run_handshake(0, 0, 0, 3u);
    run_handshake(0, 1, 0, 1u);
    run_handshake(1, 0, 0, 1u);
    run_handshake(0, 0, 1, 1u);
    late_duplicate = 0;
    publication_gap_duplicate = 1;
    run_handshake(0, 0, 0, 1u);
    CHECK(blocked_gap_attempts != 0u);
    publication_gap_duplicate = 0;
    before_start_fault = 1;
    run_handshake(0, 0, 0, 1u);
    before_start_fault = 0;
    late_duplicate = 1;
    run_handshake(0, 0, 0, 1u);
    run_handshake(0, 0, 1, 1u);
    printf("%u chassis notification checks, %u failures\n", checks, failures);
    return (failures == 0u) ? 0 : 1;
}
