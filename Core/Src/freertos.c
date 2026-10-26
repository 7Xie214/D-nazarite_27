/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Communicate.h"
#include "My_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Bluer_Da_analyz */
osThreadId_t Bluer_Da_analyzHandle;
const osThreadAttr_t Bluer_Da_analyz_attributes = {
  .name = "Bluer_Da_analyz",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for Upstairs_Task */
osThreadId_t Upstairs_TaskHandle;
const osThreadAttr_t Upstairs_Task_attributes = {
  .name = "Upstairs_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void Bluer_Da_analyz_Hander(void *argument);
void Upstairs_Task_Hander(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Bluer_Da_analyz */
  Bluer_Da_analyzHandle = osThreadNew(Bluer_Da_analyz_Hander, NULL, &Bluer_Da_analyz_attributes);

  /* creation of Upstairs_Task */
  Upstairs_TaskHandle = osThreadNew(Upstairs_Task_Hander, NULL, &Upstairs_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Bluer_Da_analyz_Hander */
/**
* @brief Function implementing the Bluer_Da_analyz thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Bluer_Da_analyz_Hander */
void Bluer_Da_analyz_Hander(void *argument)
{
  /* USER CODE BEGIN Bluer_Da_analyz_Hander */
  /* 真代码在 APP/Src/Communicate.c —— 这里只留一行转发。
   * 函数名/参数/大括号都是 CubeMX 生成区，不能删，否则重新生成会和
   * APP 里的定义撞成重复符号。
   *
   * 。 */
  Communicate_BluetoothTask();
  /* USER CODE END Bluer_Da_analyz_Hander */
}

/* USER CODE BEGIN Header_Upstairs_Task_Hander */
/**
* @brief Function implementing the Upstairs_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Upstairs_Task_Hander */
void Upstairs_Task_Hander(void *argument)
{
  /* USER CODE BEGIN Upstairs_Task_Hander */
  /* Infinite loop */
  for(;;)
  {
    upstairs_task();
    osDelay(1);
    
  }
  /* USER CODE END Upstairs_Task_Hander */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

