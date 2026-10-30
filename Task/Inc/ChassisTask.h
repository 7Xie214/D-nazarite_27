#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "cmsis_os2.h"

#define CHASSIS_TASK_START_FLAG 1u
#define CHASSIS_TASK_DONE_FLAG  2u
#define CHASSIS_TASK_ERROR_FLAG 4u

/* CMSIS-RTOS2 entry; inactive chassis selections exit without control. */
void ChassisTask(void *argument);

/* Upstairs task only; keep one request outstanding until its reply. */
osStatus_t ChassisTask_RequestFrontMove(void);
/* Consume DONE/ERROR without blocking; return zero if no reply is available. */
uint32_t ChassisTask_PollReply(void);

#endif /* CHASSIS_TASK_H */
