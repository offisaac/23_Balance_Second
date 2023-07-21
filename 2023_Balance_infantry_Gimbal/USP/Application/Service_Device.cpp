/**
	******************************************************************************
	* @file   task.cpp
	* @brief  freertos task running file.
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
#include "internal.h"
#include <Middlewares/UpperMonitor/UpperMonitor.h>
/* Private define ------------------------------------------------------------*/
TaskHandle_t IMU_Handle;
TaskHandle_t DR16_Handle;
TaskHandle_t InfantryCtrl_Handle;
TaskHandle_t DeviceIndicator_Handle;
TaskHandle_t Openlog_send_Handle;
TaskHandle_t Log_Handle;
extern float yaw_out;
/* Private function declarations ---------------------------------------------*/
void tskIMU(void *arg);
void tskDR16(void *arg);
void Device_InfantryCtrl(void *arg);
void Device_Indicator(void *arg);
void tskOpenlog_send(void *arg);
void tskLog(void *arg);
/* Function prototypes -------------------------------------------------------*/
/**
 * @brief  Initialization of device management service
 * @param  None.
 * @return None.
 */
void Service_Devices_Init(void)
{
	xTaskCreate(tskIMU, "App.IMU", Normal_Stack_Size, NULL, PriorityNormal, &IMU_Handle);
	xTaskCreate(tskDR16, "App.DR16", Normal_Stack_Size, NULL, PriorityHigh, &DR16_Handle);
	xTaskCreate(Device_InfantryCtrl, "Dev.Infantry", Huge_Stack_Size, NULL, PrioritySuperHigh, &InfantryCtrl_Handle);
	xTaskCreate(Device_Indicator, "Device.Indicator", Normal_Stack_Size, NULL, PriorityHigh, &DeviceIndicator_Handle);
	xTaskCreate(tskOpenlog_send,"App.Openlog send",Normal_Stack_Size, NULL,PriorityRealtime, &Openlog_send_Handle);
	xTaskCreate(tskLog, "App.Log", Normal_Stack_Size, NULL, PrioritySuperHigh, &Log_Handle);
}

/**
 * @brief MPU6050读取数据
 */
void tskIMU(void *arg)
{
	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	for (;;)
	{
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 1);
		/*	读取MPU6050数据	*/
		dmp_read_data(&mpu_receive);
		infantry.gimbal.MPUdata_Update(&mpu_receive);
	}
}

/**
 *	@brief	Dr16 data receive task
 */
void tskDR16(void *arg)
{
	/* Cache for Task */
	static USART_COB Rx_Package;
	/* Pre-Load for task */
	DR16.Check_Link(xTaskGetTickCount());
	/* Infinite loop */
	for (;;)
	{
		/* Enter critical */
		xSemaphoreTake(DR16_mutex, portMAX_DELAY);
		/*	等待数据	*/
		if (xQueueReceive(DR16_QueueHandle, &Rx_Package, 100) == pdPASS)
		{
			// Read Message
			DR16.DataCapture((DR16_DataPack_Typedef *)Rx_Package.address);
		}
		/*	检测遥控器连接 */
		DR16.Check_Link(xTaskGetTickCount());
		/*	判断是否连接 	 */
		if (DR16.GetStatus() != DR16_ESTABLISHED)
		{
			/**
			 * lost the remote control
			 */
			infantry.board_com.Chassis_Send_Pack1(0, 0, 0, 0);
			/* Leave critical */
			xSemaphoreGive(DR16_mutex);
			continue;
		}
		/*	更新遥控器控制	*/

		/* Leave critical */
		xSemaphoreGive(DR16_mutex);
	}
}
/**
 * @brief  步兵驱动代码
 * @param  None.
 * @return None.
 */
void Device_InfantryCtrl(void *arg)
{
	/* Cache for Task */
	/* Pre-Load for task */
	/* Infinite loop */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();

	for (;;)
	{
		/* Pass control to the next task ------------------------------------------*/
		vTaskDelayUntil(&xLastWakeTime_t, 1);

		infantry.Update_StateRequest();
		infantry.Status_Update();
		infantry.Adjust();
		infantry.Actuate();
	}
}
/**
 * @brief  操作手指示灯代码
 * @param  None.
 * @return None.
 */
void Device_Indicator(void *arg)
{
	/* Cache for Task */
	/*各种滞回比较器标志位*/
	static bool temp_full_vol, temp_mid_vol;
	/* Pre-Load for task */

	/* Infinite loop */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();

	infantry.indicator.Reset();
	vTaskDelay(50);
	for (;;)
	{
		/***************************占空比测试用********************************/
		//		delay++;
		//		delay%=100;
		//		if(delay%2==0)
		//		{
		//			infantry.indicator.Change_Singal_RGB(LED_1, 0, 255, 0, 255);
		//			infantry.indicator.Change_Singal_RGB(LED_2, 0, 0, 255, 255);
		//			infantry.indicator.Change_Singal_RGB(LED_3, 255, 255, 255, 255);
		//			infantry.indicator.Change_Singal_RGB(LED_4, 0xff, 0, 0, 255);
		//			infantry.indicator.Change_Singal_RGB(LED_5, 0, 0xff, 0, 255);
		//			infantry.indicator.Change_Singal_RGB(LED_6, 0, 0, 0xff, 255);
		//		}
		//		else
		//		{
		//			infantry.indicator.Change_Singal_RGB(LED_1, 0, 0, 0, 255);
		//			infantry.indicator.Change_Singal_RGB(LED_2, 0, 0, 0, 255);
		//			infantry.indicator.Change_Singal_RGB(LED_3, 0, 0, 0, 255);
		//			infantry.indicator.Change_Singal_RGB(LED_4, 0, 0, 0, 255);
		//			infantry.indicator.Change_Singal_RGB(LED_5, 0, 0, 0, 255);
		//			infantry.indicator.Change_Singal_RGB(LED_6, 0, 0, 0, 255);
		//		}
		/*******************************22赛季指示灯********************************/
		/*LED1（操作手界面的最右边），状态机模式*/
		if (!DR16.GetStatus())
		{
			infantry.indicator.Change_Singal_RGB(LED_1, 255, 0, 0, 255); //遥控器掉线，红
			infantry.indicator.Change_Singal_RGB(LED_2, 255, 0, 0, 255); //遥控器掉线，红
			infantry.indicator.Change_Singal_RGB(LED_3, 255, 0, 0, 255); //遥控器掉线，红
			infantry.indicator.Change_Singal_RGB(LED_5, 255, 0, 0, 255); //遥控器掉线，红
			infantry.indicator.Change_Singal_RGB(LED_6, 255, 0, 0, 255); //遥控器掉线，红
		}
		else
		{
			/*LED1,摩擦轮是否打开*/
			if (infantry.fri_state == true)
				infantry.indicator.Change_Singal_RGB(LED_1, 255, 0, 255, 255); //摩擦轮打开，紫色
			else
				infantry.indicator.Change_Singal_RGB(LED_1, 0, 0, 0, 255); //摩擦轮关闭，不亮

			/*LED2,底盘运动模式*/
			if (infantry.Get_CapMode() == NORMAL)
				infantry.indicator.Change_Singal_RGB(LED_2, 0, 0, 0, 255); //正常，不亮
			else if (infantry.Get_CapMode() == UNLIMITED)
				infantry.indicator.Change_Singal_RGB(LED_2, 255, 255, 0, 255); //超功率，黄
			else if (infantry.Get_CapMode() == ASCENT)
				infantry.indicator.Change_Singal_RGB(LED_2, 255, 0, 0, 255); //上坡，红
			else if (infantry.Get_CapMode() == LEAP)
				infantry.indicator.Change_Singal_RGB(LED_2, 255, 0, 255, 255); //飞坡，紫
			else
			{
			}
			/*LED3,电容电压显示*/
			if (infantry.indicator.Hysteresis_comparator(20, 24, infantry.cap_voltage, &temp_full_vol))
				infantry.indicator.Change_Singal_RGB(LED_3, 0, 0, 0, 255); //高电容，不亮
			else if (infantry.indicator.Hysteresis_comparator(16, 20, infantry.cap_voltage, &temp_mid_vol))
				infantry.indicator.Change_Singal_RGB(LED_3, 255, 255, 0, 255); //中电容，黄
			else
				infantry.indicator.Change_Singal_RGB(LED_3, 255, 0, 0, 255); //低电容，红

			/*LED5,弹舱盖开关*/
			if (infantry.bulletBay_state == true)
				infantry.indicator.Change_Singal_RGB(LED_5, 255, 0, 0, 255); //开弹舱盖，红
			else
				infantry.indicator.Change_Singal_RGB(LED_5, 0, 0, 0, 255); //关弹舱盖，不亮

			/*LED6,平衡步卡盲道指示*/
			if (infantry.Get_StateMachine() != KEYBOARDCTRL)
			{
				infantry.indicator.Change_Singal_RGB(LED_6, 255, 0, 0, 255); //非键鼠模式，红
			}
			else
			{
				if (infantry.turn90degrees)
					infantry.indicator.Change_Singal_RGB(LED_6, 0, 255, 255, 255); //平衡步专属，侧身，青色
				else if (infantry.rotation_state)
					infantry.indicator.Change_Singal_RGB(LED_6, 255, 0, 255, 255); //小陀螺打开，紫色
				else
					infantry.indicator.Change_Singal_RGB(LED_6, 0, 0, 0, 255); //普通，不亮

				if (infantry.self_rescue_state)
					infantry.indicator.Change_Singal_RGB(LED_6, 255, 255, 255, 255); //平衡步专属，固连自救，白
				else if (infantry.stuck_judge())
					infantry.indicator.Change_Singal_RGB(LED_6, 255, 255, 0, 255); //卡盲道，黄
			}
		}

		/*LED4,视觉瞄准*/
		if (infantry.pc_vision.PCvisionStatus == Connected)
		{
			if (infantry.Get_pcVisionMode() == NORMAL_V)
				infantry.indicator.Change_Singal_RGB(LED_4, 0, 255, 0, 255); //地面目标，绿
			else if (infantry.Get_pcVisionMode() == ROTATION_V)
				infantry.indicator.Change_Singal_RGB(LED_4, 0, 255, 255, 255); //小陀螺，青
			else if (infantry.Get_pcVisionMode() == RUNE_V)
				infantry.indicator.Change_Singal_RGB(LED_4, 255, 255, 0, 255); //打符，黄
			else
				infantry.indicator.Change_Singal_RGB(LED_4, 255, 0, 255, 255); //反打符，紫
		}
		else
			infantry.indicator.Change_Singal_RGB(LED_4, 255, 0, 0, 255); //小电脑掉线，红

		// if (!DR16.GetStatus())
		// 	infantry.indicator.Change_Singal_RGB(LED_1, 255, 0, 0, 255); //遥控器掉线，红
		// else if (infantry.Get_StateMachine() == REMOTECTRL)
		// 	infantry.indicator.Change_Singal_RGB(LED_1, 0, 0, 255, 255); //遥控器模式，蓝
		// else if (infantry.Get_StateMachine() == KEYBOARDCTRL)
		// 	infantry.indicator.Change_Singal_RGB(LED_1, 0, 0, 0, 255); //键鼠模式，不亮
		// else if (infantry.Get_StateMachine() == PERBALANCE)
		// 	infantry.indicator.Change_Singal_RGB(LED_1, 255, 255, 0, 255); //预平衡状态，黄
		// else
		// {
		// }
		// /*LED2,底盘运动模式*/
		// if (infantry.Get_CapMode() == NORMAL)
		// 	infantry.indicator.Change_Singal_RGB(LED_2, 0, 0, 0, 255); //正常，不亮
		// else if (infantry.Get_CapMode() == UNLIMITED)
		// 	infantry.indicator.Change_Singal_RGB(LED_2, 255, 255, 0, 255); //超功率，黄
		// else if (infantry.Get_CapMode() == ASCENT)
		// 	infantry.indicator.Change_Singal_RGB(LED_2, 255, 0, 0, 255); //上坡，红
		// else if (infantry.Get_CapMode() == LEAP)
		// 	infantry.indicator.Change_Singal_RGB(LED_2, 255, 0, 255, 255); //飞坡，紫
		// else
		// {
		// }
		// /*LED3,电容电压显示*/
		// if (infantry.indicator.Hysteresis_comparator(20, 24, infantry.cap_voltage, &temp_full_vol))
		// 	infantry.indicator.Change_Singal_RGB(LED_3, 0, 0, 0, 255); //高电容，不亮
		// else if (infantry.indicator.Hysteresis_comparator(16, 20, infantry.cap_voltage, &temp_mid_vol))
		// 	infantry.indicator.Change_Singal_RGB(LED_3, 255, 255, 0, 255); //中电容，黄
		// else
		// 	infantry.indicator.Change_Singal_RGB(LED_3, 255, 0, 0, 255); //低电容，红
		// /*LED4,视觉瞄准*/
		// if (infantry.pc_vision.PCvisionStatus == Connected)
		// {
		// 	if (infantry.Get_pcVisionMode() == NORMAL_V)
		// 		infantry.indicator.Change_Singal_RGB(LED_4, 0, 255, 0, 255); //地面目标，绿
		// 	else if (infantry.Get_pcVisionMode() == ROTATION_V)
		// 		infantry.indicator.Change_Singal_RGB(LED_4, 0, 255, 255, 255); //小陀螺，青
		// 	else if (infantry.Get_pcVisionMode() == RUNE_V)
		// 		infantry.indicator.Change_Singal_RGB(LED_4, 255, 255, 0, 255); //打符，黄
		// 	else
		// 		infantry.indicator.Change_Singal_RGB(LED_4, 255, 0, 255, 255); //反打符，紫
		// }
		// else
		// 	infantry.indicator.Change_Singal_RGB(LED_4, 255, 0, 0, 255); //小电脑掉线，红
		// /*LED5,弹舱盖开关*/
		// if (infantry.bulletBay_state == true)
		// 	infantry.indicator.Change_Singal_RGB(LED_5, 255, 0, 0, 255); //开弹舱盖，红
		// else
		// 	infantry.indicator.Change_Singal_RGB(LED_5, 0, 0, 0, 255); //关弹舱盖，不亮

		// /*LED6,平衡步卡盲道指示*/
		// if (infantry.self_rescue_state)
		// 	infantry.indicator.Change_Singal_RGB(LED_6, 255, 255, 255, 255); //平衡步专属，固连自救，白
		// else if (infantry.stuck_judge())
		// 	infantry.indicator.Change_Singal_RGB(LED_6, 255, 0, 0, 255); //卡盲道，红
		// else
		// 	infantry.indicator.Change_Singal_RGB(LED_6, 0, 0, 0, 255);
		// if (infantry.turn90degrees)
		// 	infantry.indicator.Change_Singal_RGB(LED_6, 0, 255, 255, 255); //平衡步专属，侧身，青色

		/*更新指示灯*/
		infantry.indicator.Update();
		/* Pass control to the next task ------------------------------------------*/
		vTaskDelayUntil(&xLastWakeTime_t, 10);
	}
}
/**
 * @brief  openlog发送
 * @param  None.
 * @return None.
 */
void tskOpenlog_send(void *arg)
{
	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	/* Infinite loop */
	for (;;)
	{
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 2);
//		if(infantry.Get_pcVisionMode() == RUNE_V && DR16.IsKeyPress(DR16_MOUSE_R))
//		{
//			openlog.Send();
//		}
//		if(yaw_out != 0)
//		{
//			openlog.Send();
//		}
	}
}
/**
 * @brief  openlog写入数据
 * @param  None.
 * @return None.
 */
void tskLog(void *arg)
{
	/* Pre-Load for task */
	vTaskDelay(1000 * 3);
	uint16_t time;
	openlog.new_file("table_%d.csv", 2);
	openlog.append_file("table_%d.csv", 2);
	openlog.record("yaw_speed_t,yaw_speed,yaw_angle,yaw_current\r");
	openlog.push_buff();
	openlog.Send();
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	/* Infinite loop */
	for (;;)
	{
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 2);
		//vTaskDelay(2);
		//time++;
//		if(infantry.Get_pcVisionMode() == RUNE_V && DR16.IsKeyPress(DR16_MOUSE_R))
//		{
//			openlog.record("%d,%d,%d,%d\r", (int16_t)infantry.gimbal.yawMotor.Out, (int16_t)infantry.gimbal.yawMotor.getSpeed(), (int16_t)infantry.gimbal.yawMotor.getAngle(),(int16_t)infantry.gimbal.yawMotor.givenCurrent);
//			openlog.push_buff();
//		}
//		if(yaw_out != 0)
//		{
//			openlog.record("%d,%d,%d,%d\r", (int16_t)infantry.gimbal.yaw_speedloop.Target, (int16_t)infantry.gimbal.yawMotor.getSpeed(), (int16_t)infantry.gimbal.yawMotor.Out,(int16_t)infantry.gimbal.yawMotor.givenCurrent);
//			openlog.push_buff();
//		}
	}
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
