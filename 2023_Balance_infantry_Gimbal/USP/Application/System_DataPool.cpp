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
QueueHandle_t USART_TxPort;					//	串口发送队列
QueueHandle_t CAN1_TxPort;					//	can1 发送队列
QueueHandle_t CAN1_RxPort;					//	can1 接收队列
QueueHandle_t CAN2_TxPort;					//	can2 发送队列
QueueHandle_t CAN2_RxPort;					//	can2 接收队列
QueueHandle_t DR16_QueueHandle;			//	dr16（串口） 接收队列

/* Semaphores */

/* Mutexes */
SemaphoreHandle_t DR16_mutex;				//	dr16互斥量

/* Notifications */

/* Other Resources -----------------------------------------------------------*/
uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE]; /*!< Receive buffer for Uart1 */
uint8_t Uart2_Rx_Buff[USART2_RX_BUFFER_SIZE]; /*!< Receive buffer for Uart2 */
uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE]; /*!< Receive buffer for Uart3 */
uint8_t Uart6_Rx_Buff[USART6_RX_BUFFER_SIZE]; /*!< Receive buffer for Uart6 */

mpu_rec_s mpu_receive;			 										//mpu6050数据
DR16_Classdef DR16;															//遥控器DR16类
InfantryCTRL_Classdef infantry;									//步兵控制类
openlog_classdef<16> openlog(Uart3_Transmit);		//openlog类
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
