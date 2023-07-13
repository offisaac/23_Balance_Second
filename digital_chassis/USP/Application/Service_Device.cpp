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
#include "Balance_Chassis.h"
#include <Middlewares/UpperMonitor/UpperMonitor.h>
/* Private define ------------------------------------------------------------*/
TaskHandle_t DjiMotor_Handle;	
TaskHandle_t LPMS_Handle;
TaskHandle_t Source_Handle;
TaskHandle_t Referee_Handle;
TaskHandle_t Referee_UI;
TaskHandle_t Openlog_send_Handle;
TaskHandle_t Log_Handle;
/* Private function declarations ---------------------------------------------*/
void tskDjiMotor(void *arg);
void tskLPMS(void *arg);
void tskSource(void *arg);
void tskRefereeRx(void *arg);
void tskRefereeUI(void *arg);
void tskOpenlog_send(void *arg);
void tskLog(void *arg);
/* Function prototypes -------------------------------------------------------*/

//LPMS_BE2_Typedef LPMS(1,1);
/**
* @brief  Initialization of device management service
* @param  None.
* @return None.
*/
void Service_Devices_Init(void)
{
  xTaskCreate(tskDjiMotor, 	"App.Motor",   Normal_Stack_Size, NULL, PriorityAboveNormal, &DjiMotor_Handle);
	xTaskCreate(tskLPMS,      "App.LPMS", Normal_Stack_Size, NULL, PriorityAboveNormal,&LPMS_Handle);
	xTaskCreate(tskSource,      "App.Source", Normal_Stack_Size, NULL, PriorityAboveNormal,&Source_Handle);
	xTaskCreate(tskRefereeRx,			"App.Referee", Normal_Stack_Size,NULL, PriorityNormal,&Referee_Handle);
	xTaskCreate(tskRefereeUI, "App.RefereeUI" , Normal_Stack_Size,NULL, PriorityBelowNormal,&Referee_UI);
	xTaskCreate(tskOpenlog_send,"App.Openlog send",Small_Stack_Size, NULL,PriorityAboveNormal, &Openlog_send_Handle);
	xTaskCreate(tskLog, 			"App.Log",    Small_Stack_Size, NULL, PriorityAboveNormal, &Log_Handle);
}

/**
 * @brief <freertos> 电源管理任务
 */
void tskSource(void *arg)
{
		Cap_Charge_Pid.SetPIDParam(2.0f, 0, 0, 200, 1000);
		Power_Limit_Pid.SetPIDParam(500.0f, 200.0f, 0, 8000, 40000);
		Power_Limit_Pid.I_SeparThresh = 8000.0f;
		balance_infantry.Power_Ctrl.Load_capChargeController(Cap_Charge_Controller);
		balance_infantry.Power_Ctrl.Load_motorLimitController(Power_Limit_Controller);
		balance_infantry.Source_Init();
		for(;;)
		{
			balance_infantry.Source_Adjust();
		}
}

/**
 * @brief <freertos> 大疆电机控制任务
 */
void tskDjiMotor(void *arg)
{
	/*	pre load for task	*/
  HAL_UART_DeInit(&huart1);
  vTaskDelay(200);
  HAL_UART_Init(&huart1);

	TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
	balance_infantry.Load_Chassis_Queue(&CAN2_TxPort, &CAN1_TxPort);
	balance_infantry.Slider_Ctrl.importQueueHander(CAN1_TxPort);
	balance_infantry.Slider_Ctrl.init();

	for(;;){
		/* wait for next circle */
    
		vTaskDelayUntil(&xLastWakeTime_t,2);
		balance_infantry.Chassis_Ctrl();
    
	}
}

/**
* @brief <freertos> 阿路比陀螺仪数据读取任务
*/
void tskLPMS(void *arg)
{
	balance_infantry.LPMS.Data_Type_Config(DATA_16BIT);
 for(;;)
 { 
    if(balance_infantry.LPMS.is_init==false)
    {
      balance_infantry.LPMS.LPMS_BE2_Init();
    }
    else if(balance_infantry.LPMS.is_init==true)
    {
      balance_infantry.LPMS.LPMS_BE2_Data_Convert();
    }
   vTaskDelay(2);
 }
}

/**
* @brief <freertos> 裁判系统数据读取任务
*/
void tskRefereeRx(void *arg)
{
	/* Pre-Load for task */
	static USART_COB* referee_pack;
	static TickType_t xLastWakeTime_t = xTaskGetTickCount();
	
	/* Infinite loop */
	for(;;){
		if(xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *) &referee_pack, 0) == pdTRUE){
		balance_infantry.Referee.unPackDataFromRF((uint8_t*)referee_pack->address, referee_pack->len);
	}
	/* Pass control to the next task */
	vTaskDelayUntil(&xLastWakeTime_t,1);
	}
}

uint32_t get_refeeretime();
/**
	*	@brief	UI绘制
	*/
void tskRefereeUI(void *arg)
{
 //起始绘制时的绘制次数。若服务器丢包率较高，该值可适当给大
 static uint8_t enable_cnt = 30;

	//初始化裁判系统
	balance_infantry.Referee.Init(&huart6, get_refeeretime);

 //图传稳定需要一段时间的延时
 vTaskDelay(500);
 balance_infantry.Referee.clean_all();

 vTaskDelay(2000);

 for (;;)
 {
   if (balance_infantry.gimbal_data.ui_reset_flag == 1) //重画UI标志位
   {
     enable_cnt = 20;
     balance_infantry.gimbal_data.ui_reset_flag = 0;
   }
   else
   {
   }
		
   if (enable_cnt)
   {
     enable_cnt--;
     //车界线、下坠标尺绘制
     //balance_infantry.Referee.Draw_Robot_Limit(180, 80, 961, 3, YELLOW, ADD_PICTURE);

     //绘制电容剩余能量
     balance_infantry.Referee.Draw_Cap_Energy(balance_infantry.Source_Cap_Voltage, 28, 12, enable_cnt, UI_X_VeryLeft, UI_Y_High); 

     //雷达站策略集部分
     balance_infantry.Referee.Draw_Auto_Lock_Range(balance_infantry.gimbal_data.vision_can_shoot, balance_infantry.auto_mode, UI_X_Middle, UI_Y_Middle, 700, 500, 2);
     //balance_infantry.Referee.HP_UI(balance_infantry.Gimbal_ID, 960, 220, 150, 30, WHITE, enable_cnt);

     //平衡步停车区域绘制
     balance_infantry.Referee.Draw_Balance_Stop_Erea(0,enable_cnt,-balance_infantry.balance_controller.current_linearSpeed.y,3.4,0.1,1);
   }
   else
   {
     //绘制电容剩余能量
     balance_infantry.Referee.Draw_Cap_Energy(balance_infantry.Source_Cap_Voltage, 28, 12, enable_cnt, UI_X_VeryLeft, UI_Y_High);
     balance_infantry.Referee.Draw_Boost(balance_infantry.gimbal_data.unlimited_state | balance_infantry.gimbal_data.leap_state, UI_X_VeryLeft, UI_Y_LittleHigh, 10, PINK);
     //balance_infantry.Referee.Draw_Bullet(balance_infantry.gimbal_data.bulletbay_state, 1800, 740, 8, GREEN); //绘制弹仓开启状态
     balance_infantry.Referee.Draw_BulletBay_Open(balance_infantry.gimbal_data.bulletbay_state,UI_X_Middle,UI_Y_LittleHigh,PINK);
     balance_infantry.Referee.Draw_Spin(balance_infantry.gimbal_data.rotation_state, 1400, 740, 10, BLUE);   //绘制小陀螺开启状态
     balance_infantry.Referee.Draw_CoolingHeat(balance_infantry.Referee.PowerHeatData.shooter_id1_17mm_cooling_heat, balance_infantry.Referee.GameRobotState.shooter_id1_17mm_cooling_limit, UI_X_Middle, UI_Y_Middle, 50, 5);

     balance_infantry.Referee.Draw_Auto_Lock_Range(balance_infantry.gimbal_data.vision_can_shoot, balance_infantry.auto_mode, UI_X_Middle, UI_Y_Middle, 700, 500, 2);

     //平衡步停车区域绘制
     balance_infantry.Referee.Draw_Balance_Stop_Erea(0,enable_cnt,-balance_infantry.balance_controller.current_linearSpeed.y,4.5,0.1,1);
     //平衡步小陀螺撞墙提示
     //balance_infantry.Referee.Draw_Rotation_Crash(4,960,720,true);

     // // infantry.Referee.HP_UI(infantry.Gimbal_ID, 960, 220, 150, 30, WHITE, enable_cnt);
     balance_infantry.Referee.Draw_Fri_State(balance_infantry.gimbal_data.fri_state, UI_X_VeryLeft, UI_Y_Middle + 75);
     // static uint8_t Data;
     // infantry.Referee.CV_ToOtherRobot(infantry.Gimbal_ID, &Data, ROBOT_COM_PACK);
//     if (!balance_infantry.gimbal_data.turn90degrees)
 //       balance_infantry.Referee.Draw_Balance_State(balance_infantry.balance_controller.current_pos.pitch/180.0f*3.14f,balance_infantry.balance_controller.current_pos.yaw/180.0f*3.14f,700,120,100,GREEN);
//      else
//        balance_infantry.Referee.Draw_Balance_State(balance_infantry.balance_controller.current_pos.pitch/180.0f*3.14f,(balance_infantry.balance_controller.current_linearSpeed.y-90)/180*3.14f,700,120,100,GREEN);
   }
   //			Referee.Engineer_AutoMode_Update(1,0,100,500,50);
   //			Referee.Draw_Fri_State(4,300,540);
   //			Referee.Draw_Cap_Energy(SourceManage.capObj.Voltage, 24, 13, enable_cnt, 380,800);
   //			Referee.Draw_Balance_State(0,1.57,950,200,100,PINK);
   
 }
}

void tskOpenlog_send(void *arg)
{
	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	/* Infinite loop */
	for(;;)
	{
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 250);
		openlog.Send();
	}
}

void tskLog(void *arg)
{
	/* Pre-Load for task */
	vTaskDelay(1000*3);
	openlog.new_file("table_%d.csv",1);
	openlog.append_file("table_%d.csv",1);
	openlog.record("Time,pow_Charge,Vcap,rotation,yaw_speed,powerBuffer,shootHeat,bulletSpeed\r");
	openlog.push_buff();
	/* Infinite loop */
	for(;;)
	{
		/* wait for next circle */
		vTaskDelay(200);
    openlog.record("%d,%d,%d,%d,%d,%d,%d,%d\r",Get_SystemTimer()/1000000,
                                              (int16_t)digital_Power.power.pow_Charge,
                                              (int8_t)digital_Power.unit_DPW_data.Vcap,
                                              (int8_t)balance_infantry.gimbal_data.rotation_state,
                                              (int16_t)(balance_infantry.balance_controller.current_angularSpeed.yaw / ratio_degree_to_rad),
                                              balance_infantry.Referee.PowerHeatData.chassis_power_buffer,
                                              balance_infantry.Referee.PowerHeatData.shooter_id1_17mm_cooling_heat,
                                              (int16_t)balance_infantry.Referee.ShootData.bullet_speed*1000.0f);
		openlog.push_buff();
	}
}


/**
 * @brief  得到s单位
 * @note
 * @param
 * @return
 * @retval  None
 */
uint32_t get_refeeretime()
{
	return xTaskGetTickCount() * 1000;
}
