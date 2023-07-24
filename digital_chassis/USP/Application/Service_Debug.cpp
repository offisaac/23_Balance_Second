/**
  **********************************************************************************
  * @file   : Service_Debug.cpp
  * @brief  : Debug support file.This file provides access ports to debug.
  **********************************************************************************
  *  
**/
/* Includes ------------------------------------------------------------------*/
#include "internal.h"
#include "Middlewares/UpperMonitor/UpperMonitor.h"

/* Private define ------------------------------------------------------------*/
TaskHandle_t Debug_Handle;
/* Private variables ---------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Task_Debug(void *arg);

/* Function prototypes -------------------------------------------------------*/
/**
* @brief  Initialize debug service based on Asuwave.
*/
void Service_Debug_Init(void)
{
  xTaskCreate(Task_Debug,		"App.Asuwave", Small_Stack_Size, NULL, PriorityBelowNormal,  &Debug_Handle);
}


/**
  * @brief  debug task
  */
void Task_Debug(void *arg)
{
  /* Cache for Task */

  /* Pre-Load for task */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();

  /* Infinite loop */
  for (;;)
  {
    /* Wait for the next cycle */
    vTaskDelayUntil(&xLastWakeTime_t, 5);

		Sent_Contorl(&huart2);
  }
}


/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
