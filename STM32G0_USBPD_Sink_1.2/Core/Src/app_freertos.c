/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max7219.h"
#include "cmsis_os2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

typedef struct {
	MAX7219_EventType event;    	     // Type of event
    MAX7219_Segments segment;		     // Segment number
    int32_t value;         			     // Numeric value (e.g., voltage, current)
    MAX7219_DecimalPoint decimal_pos;    // Decimal point position
} DISPMessage_t;

#define DISP_QUEUE_MESSAGE_MAX_SIZE      sizeof(DISPMessage_t)
#define DISP_QUEUE_PRIORITY              osPriorityLow
#define DISP_QUEUE_SIZE            		 50


/* Definitions for Receiver */
osThreadId_t DISPReceiveMsg;
const osThreadAttr_t DISPReceiveMsg_attributes = {
  .name = "DISPReceiveMsg",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};

osMessageQueueId_t DISPQueue;
const osMessageQueueAttr_t DISPQueue_attributes = {
  .name = "DISPQueue"
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void DISPReceiverTask(void * argument);

void app_freertos_create() {
	/* Create Message Queue */
	DISPQueue = osMessageQueueNew (DISP_QUEUE_SIZE, DISP_QUEUE_MESSAGE_MAX_SIZE, &DISPQueue_attributes);

	/* Create Receiver Task */
	DISPReceiveMsg = osThreadNew(DISPReceiverTask, DISPQueue, &DISPReceiveMsg_attributes); //second argument is queue handle

}

void DISPReceiverTask(void *argument) {
	osMessageQueueId_t queue = (osMessageQueueId_t) argument; // Cast to correct type
	uint32_t msg;

	for (;;)
	  {
		HAL_GPIO_TogglePin(LED_LOCK_GPIO_Port, LED_LOCK_Pin);
		if (osMessageQueueGet(queue, &msg, NULL, 200) == osOK) {
			// Process received message
			//HAL_GPIO_TogglePin(LED_LOCK_GPIO_Port, LED_LOCK_Pin);
		}
		osDelay(2000);
	  }
}

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

