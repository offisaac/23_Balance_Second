/**
  ******************************************************************************
  * @file   APP.cpp
  * @brief  Devices Application running file.
  ******************************************************************************
  * @note
  *  - Before running your devices, just do what you want ~ !
  *  - More devices or using other classification is decided by yourself ~ !
  ===============================================================================
                                    Task List
  ===============================================================================
  * <table>
  * <tr><th>Task Name     <th>Priority          <th>Frequency/Hz    <th>Stack/Byte
  * <tr><td>              <td>                  <td>                <td>
  * </table>
  *
 */

/* Includes ------------------------------------------------------------------*/
#include "app.h"
#include "internal.h"
#include <Middlewares/UpperMonitor/UpperMonitor.h>

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/*Private Function declarations -------------------------------------------------------*/
/* Function prototypes -------------------------------------------------------*/

/**
  * @brief  application device initialization task
  */
void System_Device_Init(void)
{
  /* Drivers Init */
	// timer init
  Timer_Init(&htim4, USE_HAL_DELAY);
  // can init
	CAN_Init(&hcan1, User_CAN1_RxCpltCallback);
  CAN_Init(&hcan2, User_CAN2_RxCpltCallback);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_1 | CanFifo_0 | Can_STDID | Can_DataType, 0x209, 0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_2 | CanFifo_0 | Can_STDID | Can_DataType, 0x20A, 0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_3 | CanFifo_0 | Can_STDID | Can_DataType, 0x222, 0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_4 | CanFifo_0 | Can_STDID | Can_DataType, 0x223, 0x3ff);
  // CAN_Filter_Mask_Config(&hcan2,CanFilter_15 | CanFifo_0 | Can_STDID | Can_DataType, 0x202, 0x3ff);
	// CAN_Filter_Mask_Config(&hcan2,CanFilter_16 | CanFifo_0 | Can_STDID | Can_DataType, 0x203, 0x3ff);
  CAN_Filter_Mask_Config(&hcan2, CanFilter_18 | CanFifo_0 | Can_STDID | Can_DataType, 0x140, 0x1FFFFFF8); // 9025电机
	
	// uart init
  Uart_Init(&huart3, Uart3_Rx_Buff, USART3_RX_BUFFER_SIZE, RecHandle);
	Uart_Init(&huart2, Uart2_Rx_Buff, USART2_RX_BUFFER_SIZE, User_UART2_RxCpltCallback);
	Uart_Init(&huart1, Uart1_Rx_Buff, USART1_RX_BUFFER_SIZE, User_UART1_RxCpltCallback);
	Uart_Init(&huart6, Uart6_Rx_Buff, USART6_RX_BUFFER_SIZE, Referee_recv_Callback);
  /* Modules Init */
  //初始化裁判系统
	balance_infantry.Referee.Init(&huart6, Get_SystemTimer);
  myPIDTimer::getMicroTick_regist (Get_SystemTimer);
}


/**
 * @brief application freertos init function.
 */
void System_Task_Init(void)
{
  /* Queue Init */
	CAN1_TxPort 		= xQueueCreate(4, sizeof(CAN_COB));
  CAN1_RxPort 		= xQueueCreate(4, sizeof(CAN_COB));
  CAN2_TxPort 		= xQueueCreate(4, sizeof(CAN_COB));
  CAN2_RxPort 		= xQueueCreate(4, sizeof(CAN_COB));
  USART_TxPort 		= xQueueCreate(4, sizeof(USART_COB));
	USART_RxPort		= xQueueCreate(4, sizeof(USART_COB)); 
  DR16_QueueHandle 	= xQueueCreate(2, sizeof(USART_COB));
  /* Semaphore Init */
  /* Mutex Init */
  DR16_mutex = xSemaphoreCreateMutex();
  /* Task Init */
	Service_Communication_Init();
	Service_Devices_Init();
	Service_Debug_Init();
}
