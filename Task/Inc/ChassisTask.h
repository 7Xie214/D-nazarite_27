#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#include "cmsis_os2.h"

#define CHASSIS_TASK_START_FLAG 1u
#define CHASSIS_TASK_DONE_FLAG  2u
#define CHASSIS_TASK_ERROR_FLAG 4u
#define CHASSIS_TASK_RESUME_FLAG 8u

/* CMSIS-RTOS2 entry; inactive chassis selections exit without control. */
void ChassisTask(void *argument);

/* Upstairs task only; keep one request outstanding until its reply.
 * CHASSIS_TASK_FRONT_MOVE_ENABLE=0 returns osErrorResource (zero-hold only). */
osStatus_t ChassisTask_RequestFrontMove(void);
/* Relative raw motor-rotor turns. Publishes one complete request; busy returns
 * osErrorResource without replacing the queued/active travel. Upstairs only. */
osStatus_t ChassisTask_RequestFrontMoveTurns(float left_rotor_turns, float right_rotor_turns);
osStatus_t ChassisTask_RequestFrontMoveWithPause(float left_rotor_turns, float right_rotor_turns,
                                               float pause_fraction);
osStatus_t ChassisTask_ResumeFrontMove(uint32_t move_sequence);
/* Consume DONE/ERROR without blocking; return zero if no reply is available. */
uint32_t ChassisTask_PollReply(void);
/* Upstairs观察：读取完整周期快照，返回行程编号，填入FL/FR有符号转子圈数。 */
uint32_t ChassisTask_GetFrontProgress(float actual_rotor_turns[2]);

#endif /* CHASSIS_TASK_H */
