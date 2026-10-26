#ifndef MY_TASK_H
#define MY_TASK_H

#include "FreeRTOS.h"       /* configTICK_RATE_HZ */
#include "cmsis_os2.h"      /* osKernelGetTickCount / osWaitForever */

void upstairs_task(void);

#endif /* MY_TASK_H */
