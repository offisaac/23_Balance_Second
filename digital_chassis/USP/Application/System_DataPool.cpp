/**
  ******************************************************************************
  * @file   System_DataPool.cpp
  * @brief  All used resources are contained in this file.
  ******************************************************************************
  * @note
  *  - User can define datas including variables ,structs ,and arrays in
  *    this file, which are used in deffrient tasks or services.
**/
#include "internal.h"

/* RTOS Resources ------------------------------------------------------------*/
/* Queues */
QueueHandle_t USART_TxPort;					//	���ڷ��Ͷ���
QueueHandle_t USART_RxPort;					
QueueHandle_t CAN1_TxPort;					//	can1 ���Ͷ���
QueueHandle_t CAN1_RxPort;					//	can1 ���ն���
QueueHandle_t CAN2_TxPort;					//	can2 ���Ͷ���
QueueHandle_t CAN2_RxPort;					//	can2 ���ն���
QueueHandle_t DR16_QueueHandle;			//	dr16�����ڣ� ���ն���

/* Semaphores */

/* Mutexes */
SemaphoreHandle_t DR16_mutex;				//	dr16������

/* Notifications */

/* Other Resources -----------------------------------------------------------*/
uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE]; /*!< Receive buffer for Uart1 */
uint8_t Uart2_Rx_Buff[USART2_RX_BUFFER_SIZE];
uint8_t Uart3_Rx_Buff[USART2_RX_BUFFER_SIZE]; /*!< Receive buffer for Uart3 */
uint8_t Uart6_Rx_Buff[USART2_RX_BUFFER_SIZE]; /*!< Receive buffer for Uart6 */

Balance_Infantry_Classdef balance_infantry;
openlog_classdef<16> openlog(Uart2_Transmit);
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/



