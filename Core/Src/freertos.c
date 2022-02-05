/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticSemaphore_t osStaticMutexDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
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
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Sender_task */
osThreadId_t Sender_taskHandle;
const osThreadAttr_t Sender_task_attributes = {
  .name = "Sender_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Reciver_task */
osThreadId_t Reciver_taskHandle;
uint32_t Reciver_taskBuffer[ 2048 ];
osStaticThreadDef_t Reciver_taskControlBlock;
const osThreadAttr_t Reciver_task_attributes = {
  .name = "Reciver_task",
  .cb_mem = &Reciver_taskControlBlock,
  .cb_size = sizeof(Reciver_taskControlBlock),
  .stack_mem = &Reciver_taskBuffer[0],
  .stack_size = sizeof(Reciver_taskBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Send_Sensor_Data */
osMessageQueueId_t Send_Sensor_DataHandle;
uint8_t Send_Sensor_DataBuffer[ 16 * 4 ];
osStaticMessageQDef_t Send_Sensor_DataControlBlock;
const osMessageQueueAttr_t Send_Sensor_Data_attributes = {
  .name = "Send_Sensor_Data",
  .cb_mem = &Send_Sensor_DataControlBlock,
  .cb_size = sizeof(Send_Sensor_DataControlBlock),
  .mq_mem = &Send_Sensor_DataBuffer,
  .mq_size = sizeof(Send_Sensor_DataBuffer)
};
/* Definitions for Check_IMU_Angle_Timer */
osTimerId_t Check_IMU_Angle_TimerHandle;
const osTimerAttr_t Check_IMU_Angle_Timer_attributes = {
  .name = "Check_IMU_Angle_Timer"
};
/* Definitions for USART_MUTEX */
osMutexId_t USART_MUTEXHandle;
osStaticMutexDef_t USART_MUTEXControlBlock;
const osMutexAttr_t USART_MUTEX_attributes = {
  .name = "USART_MUTEX",
  .cb_mem = &USART_MUTEXControlBlock,
  .cb_size = sizeof(USART_MUTEXControlBlock),
};
/* Definitions for Motion_Semaphore */
osSemaphoreId_t Motion_SemaphoreHandle;
osStaticSemaphoreDef_t Motion_Semaphore_ControlBlock;
const osSemaphoreAttr_t Motion_Semaphore_attributes = {
  .name = "Motion_Semaphore",
  .cb_mem = &Motion_Semaphore_ControlBlock,
  .cb_size = sizeof(Motion_Semaphore_ControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void Start_Sender_task(void *argument);
void Start_Reciver_task(void *argument);
extern void Check_Angle(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of USART_MUTEX */
  USART_MUTEXHandle = osMutexNew(&USART_MUTEX_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Motion_Semaphore */
  Motion_SemaphoreHandle = osSemaphoreNew(1, 1, &Motion_Semaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Check_IMU_Angle_Timer */
  Check_IMU_Angle_TimerHandle = osTimerNew(Check_Angle, osTimerOnce, NULL, &Check_IMU_Angle_Timer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Send_Sensor_Data */
  Send_Sensor_DataHandle = osMessageQueueNew (16, 4, &Send_Sensor_Data_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Sender_task */
  Sender_taskHandle = osThreadNew(Start_Sender_task, NULL, &Sender_task_attributes);

  /* creation of Reciver_task */
  Reciver_taskHandle = osThreadNew(Start_Reciver_task, NULL, &Reciver_task_attributes);

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
  char buff[200];

  /* Infinite loop */
  for(;;)
  {
    osDelay(200);

    // To make sure that whole text is send in one go
    osMutexAcquire(USART_MUTEXHandle, osWaitForever);

    // some long action
    for(uint16_t i = 0; i<100; i++)
    {
      uint16_t len =  (uint16_t)sprintf(buff, "Some long text %d ", i);
      HAL_UART_Transmit(&huart3, (uint8_t*)buff, len, 10);
	}

    HAL_UART_Transmit(&huart3, (uint8_t*)"\n", (uint8_t)2, 10);

    osMutexRelease(USART_MUTEXHandle);

    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_Reciver_task */
/**
* @brief Function implementing the Reciver_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Reciver_task */
void Start_Reciver_task(void *argument)
{
  /* USER CODE BEGIN Start_Reciver_task */
  char buff[64];

  /* Infinite loop */
  for(;;)
  {
    uint16_t recived_data[2];
    osStatus_t status = osMessageQueueGet(Send_Sensor_DataHandle, &recived_data[0], NULL, osWaitForever);

    if (status == osOK)
    {
      osMutexAcquire(USART_MUTEXHandle, osWaitForever);
	  uint16_t len = (uint16_t)sprintf(buff, "Roll angle = %d\r Pitch angle = %d\r", recived_data[0], recived_data[1]);
	  HAL_UART_Transmit(&huart3, (uint8_t*)buff, len, 10);
	  osMutexRelease(USART_MUTEXHandle);
	}
    else
    {
      Error_Handler();
    }
  }
  /* USER CODE END Start_Reciver_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

