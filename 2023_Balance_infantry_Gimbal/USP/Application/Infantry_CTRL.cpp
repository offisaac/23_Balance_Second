/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file Infantry_CTRL.cpp
 * @author mzh
 * @brief 步兵控制
 * @date 2022-02-15
 * @version 2.0
 * @par Change Log：
 * <table>
 * <tr><th>Date <th>Version <th>Author <th>Description
 * <tr><td>2019-06-12 <td> 1.0 <td>S.B. <td>Creator
 * </table>
 *
 ==============================================================================
 ##### How to use this driver #####
 ==============================================================================
 @note
 -#
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
* important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "Infantry_CTRL.h"
#include "stm32f4xx_hal.h"
/* Exported macros -----------------------------------------------------------*/
extern DR16_Classdef DR16;
extern QueueHandle_t CAN1_TxPort;
extern QueueHandle_t CAN2_TxPort;
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
float debug_pitch_angle = 0;
float debug_yaw_angle = 0;
/* Private type --------------------------------------------------------------*/
PerBalance_State perbalance_state;
DeathStranding_State deathstranding_state;
LostCtrl_State lostctrl_state;
RemoteCtrl_State remotectrl_state;
KeyboardCtrl_State keyboardctrl_state;
PCvisionCtrl_State pcvisionctrl_state;

/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/
/**
 * @brief 步兵pid参数初始化
 * @parma None
 * @return None
 */
void InfantryCTRL_Classdef::Infantry_Config()
{
	/*小发射pid参数*/
	booster.left_fri_speedloop.SetPIDParam(11.0f, 0.6f, 0.0f, 0, 30000.0f, 30000.0f);
	booster.right_fri_speedloop.SetPIDParam(11.0f, 0.6f, 0.0f, 0, 30000.0f, 30000.0f);
	booster.turnplate_speedloop.SetPIDParam(20.0f, 0.0f, 0.0f, 7000.0, 10000.0f, 8000.0f); // 20.0f,0.0f,0.0f,7000.0,8000.0f
	booster.turnplate_angleloop.SetPIDParam(10.0f, 0.0f, 0.02f, 0, 10000.0f, 8000.0f);		 // 10.0f,0.0f,0.05f,0,8000.0f
	/*云台pid参数*/
	gimbal.pitch_angleloop.SetPIDParam(-3000, -72000, 200, 3000, 10000, 30000);
	gimbal.yaw_angleloop.SetPIDParam(6000, 30000, -420, 3000, 150000, 30000);

	gimbal.yaw_controller.SetAngleloopParams(18, 500);
	gimbal.yaw_controller.SetSpeedloopParams(120, 1000, 300, 30000, 500);
	gimbal.yaw_controller.SetCurrentloopParams(0.25, 50, 3000, 30000, 120000);

	gimbal.pitch_controller.SetAngleloopParams(-17, 500);
	gimbal.pitch_controller.SetSpeedloopParams(80, 6000, 400, 17000, 500);
	gimbal.pitch_controller.SetCurrentloopParams(0.25, 50, 3000, 30000, 120000);

	gimbal.pitch_angleloop.I_SeparThresh = 5;
	gimbal.yaw_angleloop.I_SeparThresh = 5;

/*底盘跟随pid参数*/
#if CLASSICAL_INFANTRY
	chassisCTRL.chassis_yawAngle.SetPIDParam(50, 0, 3, 0, 10000.0f); // 去年麦轮：250,0,3,0,10000，平衡步：15,0,0,0,10000， 新舵轮：-5,0,0,0,10000
#endif
#if STEER_INFANTRY
	chassisCTRL.chassis_yawAngle.SetPIDParam(-5, 0, 0, 0, 10000.0f);
#endif
#if BALANCE_INFANTRY
	chassisCTRL.chassis_yawAngle.SetPIDParam(10, 0, 1, 0, 10000.0f, 10000.0f); // 60,0,20,0,10000.0f
	chassisCTRL.chassis_yawAngle.DeadZone = 3;
#endif
}
/**
 * @brief 预平衡状态机Handle
 * @parma None
 * @return None
 */
void PerBalance_State::Handle_State()
{
	context->state_machine = PERBALANCE;
	static uint16_t perbalance_delay_time = 800; // 预平衡延时
	/*根据底盘发来的标志位判断是否自救完成*/
	context->perbalance_delay_cnt--;
	/*预平衡状态下，云台归中*/
	context->reset_state = false;
	context->gimbal.Set_FeedbackSource(ENCODER);
	context->gimbal.Set_PitchTarget(0);
	context->gimbal.Set_YawTarget(0);
	context->gimbal.Reset_YawUpdateFlag();
	if (context->perbalance_delay_cnt < 0)
	{
		/*底盘起立*/
		context->enable_cmd = true;
		// context->enable_cmd = false;
		/*底盘起立后，云台切换回陀螺仪控制*/
		context->gimbal.Set_FeedbackSource(GYRO);
		// context->gimbal.Set_FeedbackSource(ENCODER);
		/*预平衡延时复位*/
		context->perbalance_delay_cnt = perbalance_delay_time;
		/*平衡后切换状态*/
		context->Judge_CTRLmode();
	}
}
/**
 * @brief 死亡状态机Handle
 * @parma None
 * @return None
 */
void DeathStranding_State::Handle_State()
{
	context->state_machine = DEATHSTRANDING;
	context->clear_state();
	/*进入软件复位*/
	__set_FAULTMASK(1); // 关闭所有中断
	NVIC_SystemReset();
}
/**
 * @brief 失控状态机Handle
 * @parma None
 * @return None
 */
void LostCtrl_State::Handle_State()
{
	context->state_machine = LOSTCTRL;
	context->reset_state = true;
	context->enable_cmd = false;
	context->clear_state();

	/* 若遥控马上开了又关，则重新开遥控后会缩短整个延时时间 */
	context->perbalance_delay_cnt = 800;
}

//单环参数
float normal_pitch_angle_kp = -2500;
float normal_pitch_angle_ki = -40000; // 320
float normal_pitch_angle_kd = 175;
float normal_pitch_angle_imax = 3000; // 16
float normal_pitch_angle_pimax = 100000;
float normal_pitch_angle_Ist = 400;
float normal_yaw_angle_kp = 6000;
float normal_yaw_angle_ki = 30000;
float normal_yaw_angle_kd = -420;
float normal_yaw_angle_imax = 3000;
float normal_yaw_angle_pimax = 150000;
float normal_yaw_angle_Ist = 5;
//全补偿参数
float fn_pitchcurrent_kp = 0.25;
float fn_pitchcurrent_ki = 50;
float fn_pitchcurrent_imax = 3000;
float fn_pitchcurrent_omax = 26000;
float fn_pitchspeed_kp = 75; // 150
float fn_pitchspeed_ki = 1000;
float fn_pitchspeed_imax = 300;
float fn_pitchangle_kp = -15; // 25

float fn_yawcurrent_kp = 0.25;
float fn_yawcurrent_ki = 50;
float fn_yawcurrent_imax = 3000;
float fn_yawcurrent_omax = 26000;
float fn_yawspeed_kp = 140; // 150
float fn_yawspeed_ki = 1000;
float fn_yawspeed_imax = 0;
float fn_yawangle_kp = 13; // 25
/**
 * @brief 遥控控制状态机Handle
 * @parma None
 * @return None
 */
void RemoteCtrl_State::Handle_State()
{
	context->state_machine = REMOTECTRL;
	static uint8_t temp_rotationState = 1;								 // 底盘判断标志位
	static uint8_t temp_friWheel = 1, temp_laserState = 1; // 小发射判断标志位
	/*底盘*/
	context->y_data = DR16.Get_LY_Norm();
	context->x_data = DR16.Get_LX_Norm();
	context->y_back_data = 0;
	context->x_back_data = 0;
/*舵轮关闭底盘跟随*/
#if STEER_INFANTRY
	context->chassisCTRL.ChassisFollowOn(false);
	context->chassis_follow_flag = 0;
#endif

	if (DR16.GetS1() == DR16_SW_DOWN)
	{
		switch (DR16.GetS2())
		{
		case DR16_SW_UP:
			context->cap_mode = LEAP;
			// context->cap_mode = UNLIMITED;
			break;
		case DR16_SW_MID:
			context->cap_mode = NORMAL;
			temp_rotationState = false;
			break;
		case DR16_SW_DOWN:
			context->rotation_state = context->LogicJudge(context->rotation_state, DR16.GetS2() == DR16_SW_DOWN, &temp_rotationState);
			break;
		default:
			break;
		}
	}
	/*云台*/
	context->pitch_data = DR16.Get_RY_Norm() * PITCH_SCALE_REMOTE;
	context->yaw_data = -DR16.Get_RX_Norm() * YAW_SCALE_REMOTE;
	//	context->pitch_data = debug_pitch_angle;
	//	context->yaw_data = debug_yaw_angle;
	/*小发射*/
	if (DR16.GetS1() == DR16_SW_UP)
	{
		switch (DR16.GetS2())
		{
		case DR16_SW_UP:
			if (!context->fri_state) // 未开摩擦轮时为弹舱盖开关
			{
				context->bulletBay_state = true;
				context->gimbal.Set_PitchTarget(0); // pitch复位
			}
			else // 开启时为拨盘开关
			{
				context->turnplate_state = true;
			}
			break;
		case DR16_SW_MID:
			context->bulletBay_state = false;
			context->turnplate_state = false;
			temp_laserState = false;
			temp_friWheel = false; // 摩擦轮标志位复位
			break;
		case DR16_SW_DOWN:
			context->laser_state = context->LogicJudge(context->laser_state, DR16.GetS2() == DR16_SW_DOWN, &temp_laserState);
			context->fri_state = context->LogicJudge(context->fri_state, DR16.GetS2() == DR16_SW_DOWN, &temp_friWheel);
			break;
		default:
			break;
		}
	}
	context->gimbal.pitch_angleloop.SetPIDParam(normal_pitch_angle_kp, normal_pitch_angle_ki, normal_pitch_angle_kd, normal_pitch_angle_imax, normal_pitch_angle_pimax, 30000);
	context->gimbal.yaw_angleloop.SetPIDParam(normal_yaw_angle_kp, normal_yaw_angle_ki, normal_yaw_angle_kd, normal_yaw_angle_imax, normal_yaw_angle_pimax, 30000);
	context->gimbal.pitch_angleloop.I_SeparThresh = normal_pitch_angle_Ist;
	context->gimbal.yaw_angleloop.I_SeparThresh = normal_yaw_angle_Ist;

	context->gimbal.yaw_controller.SetAngleloopParams(fn_yawangle_kp, 500);
	context->gimbal.yaw_controller.SetSpeedloopParams(fn_yawspeed_kp, fn_yawspeed_ki, fn_yawspeed_imax, 30000, 500);
	context->gimbal.yaw_controller.SetCurrentloopParams(fn_yawcurrent_kp, fn_yawcurrent_ki, fn_yawcurrent_imax, 30000, 120000);

	context->gimbal.pitch_controller.SetAngleloopParams(fn_pitchangle_kp, 500);
	context->gimbal.pitch_controller.SetSpeedloopParams(fn_pitchspeed_kp, fn_pitchspeed_ki, fn_pitchspeed_imax, 30000, 500);
	context->gimbal.pitch_controller.SetCurrentloopParams(fn_pitchcurrent_kp, fn_pitchcurrent_ki, fn_pitchcurrent_imax, 30000, 120000);
}
/**
 * @brief 键盘控制状态机Handle
 * @parma None
 * @return None
 */
void KeyboardCtrl_State::Handle_State()
{
	context->state_machine = KEYBOARDCTRL;
	static uint8_t leap_state, unlimited_state, temp_unlimited, temp_leap; // 底盘判断标志位
	static uint8_t last_leap, last_unlimited;
	static uint8_t temp_bulletBay;										// 小发射判断标志位
	static uint8_t temp_turnBack, temp_turn90degrees; // 云台判断标志位
	static uint8_t temp_ws, temp_ad;									//辅助车身方向检测
	static uint8_t temp_self_rescue;
	static int16_t gg_mode, gimbal_gg_delay_cnt = 0; // 云台延时复位，确保gg_flag下发到底盘

	/*刷新UI*/
	if (DR16.IsKeyPress(DR16_KEY_F) && DR16.IsKeyPress(DR16_KEY_CTRL))
	{
		context->ui_reset_flag = 1;
	}
	else
	{
		context->ui_reset_flag = 0;
	}
	/*GGMode*/
	if (DR16.IsKeyPress(DR16_KEY_G) && DR16.IsKeyPress(DR16_KEY_CTRL))
	{
		gg_mode = 1;
	}
	if (gg_mode)
	{
		context->gg_flag = 1;
		if (gimbal_gg_delay_cnt >= 200)
		{
			/*进入软件复位*/
			__set_FAULTMASK(1); // 关闭所有中断
			NVIC_SystemReset();
			gimbal_gg_delay_cnt = 0;
			gg_mode = 0;
		}
		gimbal_gg_delay_cnt++;
	}
	else
		context->gg_flag = 0;

	/*底盘*/
	context->y_data = DR16.IsKeyPress(DR16_KEY_W);
	context->x_data = DR16.IsKeyPress(DR16_KEY_D);
	context->y_back_data = DR16.IsKeyPress(DR16_KEY_S);
	context->x_back_data = DR16.IsKeyPress(DR16_KEY_A);
	context->rotation_state = DR16.IsKeyPress(DR16_KEY_SHIFT);

	//检测信号上升沿并且赋值
	if (context->LogicJudge(0, DR16.IsKeyPress(DR16_KEY_W) || DR16.IsKeyPress(DR16_KEY_S), &temp_ws))
	{
		context->turn90degrees = false;
		context->turn_way = 1;
	}
	else if (context->LogicJudge(0, DR16.IsKeyPress(DR16_KEY_A) || DR16.IsKeyPress(DR16_KEY_D), &temp_ad))
	{
		if (context->turn90degrees == false)
		{
			if (DR16.IsKeyPress(DR16_KEY_A))
				context->turn_way = 1;
			else if (DR16.IsKeyPress(DR16_KEY_D))
				context->turn_way = 0;
		}
		context->turn90degrees = true;
	}
	/*按V，平衡步侧身90°*/
	context->turn90degrees = context->LogicJudge(context->turn90degrees, DR16.IsKeyPress(DR16_KEY_V), &temp_turn90degrees);
	if (context->turn90degrees && context->self_rescue_state == false)
	{
		// context->y_data = DR16.IsKeyPress(DR16_KEY_D);
		// context->y_back_data = DR16.IsKeyPress(DR16_KEY_A);
		// context->gimbal.yawMotor.setEncoderOffset((YAW_OFFSET + 2048) % 8192); // 确保不会超出8192
		if (context->turn_way == 1)
			context->chassisCTRL.chassis_yawAngle.Target = -90;
		else
			context->chassisCTRL.chassis_yawAngle.Target = 90;
	}
	else
	{
		// context->gimbal.yawMotor.setEncoderOffset(YAW_OFFSET);

		context->chassisCTRL.chassis_yawAngle.Target = 0;
	}

	/*运动策略*/
	leap_state = context->LogicJudge(leap_state, DR16.IsKeyPress(DR16_KEY_Z), &temp_leap);
	unlimited_state = context->LogicJudge(unlimited_state, DR16.IsKeyPress(DR16_KEY_C), &temp_unlimited);

	if (context->board_com.rx_pack2.cap_voltage < 16.0f) // 低电容模式
	{
		context->cap_mode = NORMAL;
	}
	else
	{
		if (unlimited_state && context->self_rescue_state) // 防止与ctrl+c键位耦合
		{
			context->cap_mode = NORMAL;
		}
		else
		{
			// context->cap_mode = (E_CapMode)(leap_state << 1 | unlimited_state);
			if (leap_state != last_leap)
			{
				context->cap_mode = (E_CapMode)(leap_state << 1);
				unlimited_state = false;
			}
			else if (unlimited_state != last_unlimited)
			{
				context->cap_mode = (E_CapMode)(unlimited_state);
				leap_state = false;
			}
		}
	}
	last_leap = leap_state;
	last_unlimited = unlimited_state;

	/*按ctrl+s，滑块软件复位*/
	context->sliding_remake = (DR16.IsKeyPress(DR16_KEY_Z) && DR16.IsKeyPress(DR16_KEY_CTRL));
	/*按ctrl+c，开启固连自救*/
	context->self_rescue_state = context->LogicJudge(context->self_rescue_state, DR16.IsKeyPress(DR16_KEY_C) && DR16.IsKeyPress(DR16_KEY_CTRL), &temp_self_rescue);

	/*按X云台旋转180°*/
	if (context->LogicJudge(0, DR16.IsKeyPress(DR16_KEY_X), &temp_turnBack))
	{
		context->gimbal.Set_YawTarget(context->gimbal.Get_YawTarget() - 180);
	}

	if (DR16.IsKeyPress(DR16_KEY_G)) // 关闭摩擦轮
	{
		context->fri_state = false;
		// context->laser_state = false;
	}
	else if (DR16.IsKeyPress(DR16_KEY_F) && !DR16.IsKeyPress(DR16_KEY_CTRL)) // 开启摩擦轮
	{
		context->fri_state = true;
		// context->laser_state = true;
	}

	context->laser_state = context->fri_state;

	/*开启弹舱盖关闭战斗模式*/
	context->bulletBay_state = context->LogicJudge(context->bulletBay_state, DR16.IsKeyPress(DR16_KEY_B), &temp_bulletBay);
	if (context->bulletBay_state)
	{
		/*开弹仓时，pitch轴复位*/
		context->gimbal.Set_PitchTarget(0);
		context->fri_state = false;
	}
	/*开启小陀螺进入战斗状态*/
	else if (context->rotation_state)
	{
		context->bulletBay_state = false;
		context->fri_state = true;
		context->laser_state = true;
	}

	/*切换各种视觉模式*/
	/**
	 * 视觉标志位
	 * 地面目标 00
	 * 打符		 01
	 * 反打符	 10
	 * 小陀螺	11
	 */
	context->booster.auto_fire = 0; //打符接管发射置零
	if (DR16.IsKeyPress(DR16_KEY_R) && DR16.IsKeyPress(DR16_KEY_CTRL))
	{
		context->pc_vision_mode = AGAINST_RUNE_V; // 反打符
		context->vision_mode_flag1 = 1;
		context->vision_mode_flag2 = 0;
	}
	else if (DR16.IsKeyPress(DR16_KEY_R) && !DR16.IsKeyPress(DR16_KEY_CTRL))
	{
		context->pc_vision_mode = RUNE_V; // 打符
		context->vision_mode_flag1 = 0;
		context->vision_mode_flag2 = 1;
	}
	else if (DR16.IsKeyPress(DR16_KEY_Q))
	{
		context->pc_vision_mode = NORMAL_V; // 打地面目标
		context->vision_mode_flag1 = 0;
		context->vision_mode_flag2 = 0;
	}
	else if (DR16.IsKeyPress(DR16_KEY_E))
	{
		context->pc_vision_mode = ROTATION_V; // 打小陀螺
		context->vision_mode_flag1 = 1;
		context->vision_mode_flag2 = 1;
	}

	/*云台*/
	if (context->pc_vision.PackFromVisionUnion.PackFromVision.target_mode == NO_ARMOR || DR16.IsKeyPress(DR16_MOUSE_R) == false)
	{
		context->pitch_data = -DR16.Get_MouseY_Norm() * PITCH_SCALE_KEYBOARD;
		context->yaw_data = -DR16.Get_MouseX_Norm() * YAW_SCALE_KEYBOARD;
	}
	else
	{
		context->pitch_data = 0;
		context->yaw_data = 0;
	}

	if (DR16.IsKeyPress(DR16_MOUSE_R))
	{
		/*右键切换至视觉状态机*/
		context->TransitionTo(&pcvisionctrl_state);
		context->current_state->Handle_State();
	}
	else
	{
		context->chassisCTRL.ChassisFollowOn(true);
		context->turnplate_state = DR16.IsKeyPress(DR16_MOUSE_L);
		context->booster.Set_TurnplateFrq(10); // 手瞄使用中等射频10
		context->pc_vision.PackToVision.R_key = false;
		if (DR16.IsKeyPress(DR16_MOUSE_L))
		{
			context->fri_state = true;
		}

		context->gimbal.pitch_angleloop.SetPIDParam(normal_pitch_angle_kp, normal_pitch_angle_ki, normal_pitch_angle_kd, normal_pitch_angle_imax, normal_pitch_angle_pimax, 30000);
		context->gimbal.yaw_angleloop.SetPIDParam(normal_yaw_angle_kp, normal_yaw_angle_ki, normal_yaw_angle_kd, normal_yaw_angle_imax, normal_yaw_angle_pimax, 30000);
		context->gimbal.pitch_angleloop.I_SeparThresh = normal_pitch_angle_Ist;
		context->gimbal.yaw_angleloop.I_SeparThresh = normal_yaw_angle_Ist;

		context->gimbal.yaw_controller.SetAngleloopParams(fn_yawangle_kp, 500);
		context->gimbal.yaw_controller.SetSpeedloopParams(fn_yawspeed_kp, fn_yawspeed_ki, fn_yawspeed_imax, 30000, 400);
		context->gimbal.yaw_controller.SetCurrentloopParams(fn_yawcurrent_kp, fn_yawcurrent_ki, fn_yawcurrent_imax, 30000, 120000);

		context->gimbal.pitch_controller.SetAngleloopParams(fn_pitchangle_kp, 500);
		context->gimbal.pitch_controller.SetSpeedloopParams(fn_pitchspeed_kp, fn_pitchspeed_ki, fn_pitchspeed_imax, 30000, 500);
		context->gimbal.pitch_controller.SetCurrentloopParams(fn_pitchcurrent_kp, fn_pitchcurrent_ki, fn_pitchcurrent_imax, 30000, 120000);
	}
}
/**
 * @brief 视觉控制状态机Handle，视觉操控放在uart1中断里，防止延时
 * @parma None
 * @return None
 */
//单环参数
// 视觉模式pid参数（非打符模式）
float pitch_angle_kp = -2700;
float pitch_angle_ki = -40000; // 320
float pitch_angle_kd = 190;
float pitch_angle_imax = 4500; // 16
float pitch_angle_pimax = 100000;
float pitch_angle_outmax = 30000;
float pitch_angle_Ist = 400;
float yaw_angle_kp = 3000;
float yaw_angle_ki = 20000;
float yaw_angle_kd = -300;
float yaw_angle_imax = 1500;
float yaw_angle_pimax = 150000;
float yaw_angle_outmax = 22000;
float yaw_angle_Ist = 400;
// 视觉模式pid参数（打符模式）
float rune_pitch_angle_kp = -3700;
float rune_pitch_angle_ki = -40000; // 320
float rune_pitch_angle_kd = 230;
float rune_pitch_angle_imax = 5000; // 16
float rune_pitch_angle_pimax = 100000;
float rune_pitch_angle_outmax = 30000;
float rune_pitch_angle_Ist = 400;
float rune_yaw_angle_kp = 4200;
float rune_yaw_angle_ki = 30000;
float rune_yaw_angle_kd = -300;
float rune_yaw_angle_imax = 3000;
float rune_yaw_angle_pimax = 100000;
float rune_yaw_angle_outmax = 30000;
float rune_yaw_angle_Ist = 400;

//全补偿参数
//非打符
float fc_pitchcurrent_kp = 0.25;
float fc_pitchcurrent_ki = 50;
float fc_pitchcurrent_imax = 3000;
float fc_pitchcurrent_omax = 30000;
float fc_pitchspeed_kp = 80; // 150
float fc_pitchspeed_ki = 1200;
float fc_pitchspeed_imax = 5000;
float fc_pitchangle_kp = -16; // 25

float fc_current_kp = 0.25;
float fc_current_ki = 50;
float fc_current_imax = 3000;
float fc_current_omax = 26000;
float fc_speed_kp = 100; // 150
float fc_speed_ki = 1000;
float fc_speed_imax = 800;
float fc_angle_kp = 25; // 25
//打符
float fr_pitchcurrent_kp = 0.25;
float fr_pitchcurrent_ki = 50;
float fr_pitchcurrent_imax = 3000;
float fr_pitchcurrent_omax = 26000;
float fr_pitchspeed_kp = 80; // 150
float fr_pitchspeed_ki = 6000;
float fr_pitchspeed_imax = 400;
float fr_pitchangle_kp = -17; // 25
float fr_current_kp = 0.25;
float fr_current_ki = 50;
float fr_current_imax = 3000;
float fr_current_omax = 30000;
float fr_speed_kp = 150; // 150
float fr_speed_ki = 1000;
float fr_speed_imax = 800;
float fr_angle_kp = 25; // 25
void PCvisionCtrl_State::Handle_State()
{
	static uint8_t last_can_shoot = 0;

	// context->fri_state = true;
	context->laser_state = false; // 自瞄关闭激光
	// context->bulletBay_state = false; // 自瞄关闭弹舱盖
	if (DR16.IsKeyPress(DR16_MOUSE_L))
	{
		context->fri_state = true;
	}
	/*激光，若识别到装甲板且不是反打符模式，则关闭红外激光*/
	if (context->pc_vision_mode != AGAINST_RUNE_V)
	{
		context->laser_state = false;
	}
	else
	{
		context->laser_state = true;
	}
	// 按右键下发标志位给视觉
	context->pc_vision.PackToVision.R_key = true;
	/*视觉模式切换pid*/
	if (context->pc_vision_mode == RUNE_V || context->pc_vision_mode == AGAINST_RUNE_V)
	{
		context->gimbal.pitch_angleloop.SetPIDParam(rune_pitch_angle_kp, rune_pitch_angle_ki, rune_pitch_angle_kd, rune_pitch_angle_imax, rune_pitch_angle_pimax, rune_pitch_angle_outmax);
		context->gimbal.yaw_angleloop.SetPIDParam(rune_yaw_angle_kp, rune_yaw_angle_ki, rune_yaw_angle_kd, rune_yaw_angle_imax, rune_yaw_angle_pimax, rune_yaw_angle_outmax);
		context->fri_state = true;
		context->gimbal.pitch_angleloop.I_SeparThresh = pitch_angle_Ist;
		context->gimbal.yaw_angleloop.I_SeparThresh = yaw_angle_Ist;

		context->gimbal.yaw_controller.SetAngleloopParams(fr_angle_kp, 500);
		context->gimbal.yaw_controller.SetSpeedloopParams(fr_speed_kp, fr_speed_ki, fr_speed_imax, 30000, 400);
		context->gimbal.yaw_controller.SetCurrentloopParams(fr_current_kp, fr_current_ki, fr_current_imax, 30000, 120000);
	
		context->gimbal.pitch_controller.SetAngleloopParams(fr_pitchangle_kp, 500);
		context->gimbal.pitch_controller.SetSpeedloopParams(fr_pitchspeed_kp, fr_pitchspeed_ki, fr_pitchspeed_imax, 30000, 500);
		context->gimbal.pitch_controller.SetCurrentloopParams(fr_pitchcurrent_kp, fr_pitchcurrent_ki, fr_pitchcurrent_imax, 30000, 120000);
	}
	else
	{
		context->gimbal.pitch_angleloop.SetPIDParam(pitch_angle_kp, pitch_angle_ki, pitch_angle_kd, pitch_angle_imax, pitch_angle_pimax, pitch_angle_outmax);
		context->gimbal.yaw_angleloop.SetPIDParam(yaw_angle_kp, yaw_angle_ki, yaw_angle_kd, yaw_angle_imax, yaw_angle_pimax, yaw_angle_outmax);
		context->gimbal.pitch_angleloop.I_SeparThresh = rune_pitch_angle_Ist;
		context->gimbal.yaw_angleloop.I_SeparThresh = rune_yaw_angle_Ist;

		context->gimbal.yaw_controller.SetAngleloopParams(fc_angle_kp, 500);
		context->gimbal.yaw_controller.SetSpeedloopParams(fc_speed_kp, fc_speed_ki, fc_speed_imax, 30000, 400);
		context->gimbal.yaw_controller.SetCurrentloopParams(fc_current_kp, fc_current_ki, fc_current_imax, 30000, 120000);

		context->gimbal.pitch_controller.SetAngleloopParams(fc_pitchangle_kp, 500);
		context->gimbal.pitch_controller.SetSpeedloopParams(fc_pitchspeed_kp, fc_pitchspeed_ki, fc_pitchspeed_imax, 30000, 500);
		context->gimbal.pitch_controller.SetCurrentloopParams(fc_pitchcurrent_kp, fc_pitchcurrent_ki, fc_pitchcurrent_imax, 30000, 120000);
	}

	/*小发射*/
	if (context->pc_vision_mode == RUNE_V) // 打符模式
	{
		/*打符关闭底盘跟随*/
		context->chassisCTRL.ChassisFollowOn(false);

		context->pc_vision.shoot_mode = context->pc_vision.PackFromVisionUnion.PackFromVision.shoot_mode;

		if (context->pc_vision.shoot_mode - context->pc_vision.last_shoot_mode == 1)
		{
			if (context->pc_vision.have_fire == 0)
			{
				context->booster.auto_fire = 1;
				context->pc_vision.have_fire = 1;
				context->pc_vision.count = 0;
			}
		}
		else if (context->pc_vision.shoot_mode - context->pc_vision.last_shoot_mode == 0)
		{
			context->booster.auto_fire = 0;
		}

		if (context->pc_vision.have_fire == 1)
		{
			context->pc_vision.count++;
		}
		if (context->pc_vision.count >= 200)
		{
			context->pc_vision.have_fire = 0;
		}

		context->pc_vision.last_shoot_mode = context->pc_vision.shoot_mode;
		// 操作手可接管打符
		context->turnplate_state = DR16.IsKeyPress(DR16_MOUSE_L);
	}
	else
	{
		context->booster.auto_fire = 0;

		if (context->pc_vision_mode == ROTATION_V)
		{
			if (context->pc_vision.PackFromVisionUnion.PackFromVision.shoot_mode != CAN_SHOOT && last_can_shoot == CAN_SHOOT)
			{
				context->vision_can_shoot_count = VISION_CAN_SHOOT_DELAY;
			}
			else
			{
				context->vision_can_shoot_count--;
			}
			if (context->vision_can_shoot_count > 0)
			{
				context->fact_vision_can_shoot = CAN_SHOOT;
			}
			else
			{
				context->fact_vision_can_shoot = context->pc_vision.PackFromVisionUnion.PackFromVision.shoot_mode;
			}
			context->vision_can_shoot_count = std_lib::constrain(context->vision_can_shoot_count, (int16_t)0, (int16_t)VISION_CAN_SHOOT_DELAY);
		}
		else
		{
			context->fact_vision_can_shoot = context->pc_vision.PackFromVisionUnion.PackFromVision.shoot_mode;
		}

		// 其他模式由视觉控制，操作手可以介入
		context->turnplate_state = DR16.IsKeyPress(DR16_MOUSE_L) || ((context->fact_vision_can_shoot == CAN_SHOOT) ? 1 : 0);

		context->booster.Set_TurnplateFrq(15); // 打地面目标使用高射频
		context->chassisCTRL.ChassisFollowOn(true);
	}
	last_can_shoot = context->pc_vision.PackFromVisionUnion.PackFromVision.shoot_mode;
}
/**
 * @brief 标志位、参数下发给云台、小发射、视觉、底盘控制
 * @parma None
 * @return None
 */
void InfantryCTRL_Classdef::Status_Update()
{
	// gimbal.Set_FeedbackSource(ENCODER);
	gimbal.Status_Update(&pitch_data, &yaw_data, &reset_state);

	booster.Status_Update(&fri_state,
												&laser_state,
												&turnplate_state,
												&bulletBay_state,
												&reset_state,
												&board_com.rx_pack1.bullet_speed,
												&board_com.rx_pack1.booster_heat,
												&board_com.rx_pack1.cooling_rate,
												&board_com.rx_pack1.heat_limit,
												&board_com.rx_pack2.booster_maxspeed);
	pc_vision.Status_Update(pc_vision_mode,
													gimbal.Get_PitchCurrent(),
													gimbal.Get_YawTotal(),
													gimbal.Get_Angular_Velocity_Pitch(),
													gimbal.Get_Angular_Velocity_Yaw(),
													&board_com.rx_pack2.booster_maxspeed,
													&board_com.rx_pack2.robot_id
													/*&board_com.big_rune_flag*/);
	chassisCTRL.Status_Update(gimbal.Get_YawMotorAngle(),
														y_data,
														x_data,
														y_back_data,
														x_back_data,
														&reset_state,
														&rotation_state,
														&keyboard_state,
														&bulletBay_state);
}
/**
 * @brief 更新状态并提交状态切换请求
 * @parma None
 * @return None
 */
void InfantryCTRL_Classdef::Update_StateRequest()
{
	if (DR16.GetStatus() != DR16_ESTABLISHED)
	{
		TransitionTo(&lostctrl_state);
	}
#if BALANCE_INFANTRY
	else if (current_state == &lostctrl_state)
	{
		TransitionTo(&perbalance_state);
	}
#endif
	else if (current_state != &perbalance_state)
	{
		Judge_CTRLmode();
	}
	/* 将控制交给状态处理机 */
	current_state->Handle_State();
}
/**
 * @brief 云台、小发射等对传入的数据进行计算
 * @parma None
 * @return None
 */
void InfantryCTRL_Classdef::Adjust()
{
	gimbal.Adjust();
	chassisCTRL.Adjust();
	booster.Adjust();
}
/**
 * @brief 控制量下发到电机
 * @parma None
 * @return None
 */
void InfantryCTRL_Classdef::Actuate()
{
	static Motor_CAN_COB can1_cob;
	static Motor_CAN_COB can2_cob;
	/*云台*/
	gimbal.Pack_CAN(1, &can1_cob);
	gimbal.Pack_CAN(2, &can2_cob);
	xQueueSend(CAN1_TxPort, &can1_cob.Id1ff, 0);
	xQueueSend(CAN2_TxPort, &can2_cob.Id1ff, 0);
	/*小发射*/
	booster.Pack_CAN(&can1_cob);
	xQueueSend(CAN1_TxPort, &can1_cob.Id200, 0);
	/*视觉*/
	pc_vision.SendGimbleStatus();
	/*底盘板间通信*/
	//enable_cmd = false;
#if BALANCE_INFANTRY
	board_com.Set_BalanceInfanty_Flag(DR16.GetStatus(),
																		cap_mode == UNLIMITED,
																		cap_mode == ASCENT,
																		cap_mode == LEAP,
																		chassisCTRL.rotationState,
																		bulletBay_state,
																		ui_reset_flag,
																		vision_mode_flag1,
																		vision_mode_flag2,
																		self_rescue_state,
																		enable_cmd,
																		sliding_remake,
																		turn90degrees,
																		gg_flag,
																		pc_vision.PackFromVisionUnion.PackFromVision.shoot_mode == CAN_SHOOT,
																		fri_state);
#endif
	board_com.Chassis_Send_Pack1(chassisCTRL.GetSpeed_Y(),
															 chassisCTRL.GetSpeed_X(),
															 chassisCTRL.GetSpeed_Z(),
															 reset_state);
	board_com.Chassis_Send_Pack2(chassisCTRL.chassis_yaw_current_full);
}
/**
 * @brief 更新电机、板间通信数据
 * @parma None
 * @return None
 */
void InfantryCTRL_Classdef::Update_Data(CAN_COB CAN_RxMsg)
{
	/*云台*/
	if (gimbal.pitchMotor.CheckID(CAN_RxMsg.ID))
	{
		gimbal.pitchMotor.update(CAN_RxMsg.Data);
	}
	else if (gimbal.yawMotor.CheckID(CAN_RxMsg.ID))
	{
		gimbal.yawMotor.update(CAN_RxMsg.Data);
	}
	/*小发射*/
	else if (booster.left_friWheel.CheckID(CAN_RxMsg.ID))
	{
		booster.left_friWheel.update(CAN_RxMsg.Data);
	}
	else if (booster.right_friWheel.CheckID(CAN_RxMsg.ID))
	{
		booster.right_friWheel.update(CAN_RxMsg.Data);
	}
	else if (booster.turnplateMotor.CheckID(CAN_RxMsg.ID))
	{
		booster.turnplateMotor.update(CAN_RxMsg.Data);
	}
	/*板间通信*/
	else if (CAN_RxMsg.ID == GIMBAL_RXPACK1)
	{
		board_com.Booster_Receive(CAN_RxMsg.Data);
	}
	else if (CAN_RxMsg.ID == GIMBAL_RXPACK2)
	{
		board_com.Chassis_Receive(CAN_RxMsg.Data);
		cap_voltage = (float)board_com.rx_pack2.cap_voltage / 7.f;
		switch (board_com.rx_pack2.source_power_max)
		{
		case 50:
			chassisCTRL.chassis_yawAngle.SetPIDParam(15, 0, 3., 0, 10000.0f, 500.0f);
			break;
		case 60:
			chassisCTRL.chassis_yawAngle.SetPIDParam(15, 0, 3., 0, 10000.0f, 500.0f);
			break;
		case 80:
			chassisCTRL.chassis_yawAngle.SetPIDParam(15, 0, 3., 0, 10000.0f, 500.0f);
			break;
		case 100:
			chassisCTRL.chassis_yawAngle.SetPIDParam(15, 0, 3., 0, 10000.0f, 500.0f);
			break;
		default:
			chassisCTRL.chassis_yawAngle.SetPIDParam(15, 0, 3., 0, 10000.0f, 500.0f);
			break;
		}
	}
}
/**
 * @brief 判断按键是否按下，同时修改受控制量的状态
 * @parma state：受控制的量
 *				 ctrl：按键
 *				 flag：标志位
 * @return state
 */
uint8_t InfantryCTRL_Classdef::LogicJudge(uint8_t state, uint8_t ctrl, uint8_t *flag)
{
	if (*flag == 0 && ctrl == 1)
	{
		*flag = 1;
		if (state == 1)
			return 0;
		else if (state == 0)
			return 1;
	}

	if (ctrl == 0)
	{
		*flag = 0;
	}
	return state;
}
/**
 * @brief 判断键鼠、遥控模式、死亡模式
 * @parma None
 * @return None
 */
void InfantryCTRL_Classdef::Judge_CTRLmode()
{
	reset_state = false;
	/*通过can2是否进入中断来判断是否死亡*/
	death_delay_cnt++; // 清除标志位在can1中断里
	if (death_delay_cnt >= 1000)
	{
		TransitionTo(&deathstranding_state);
		return;
	}
	/*判断键鼠、遥控模式*/
	if (DR16.GetS1() == DR16_SW_MID)
	{
		keyboard_state = true;
		TransitionTo(&keyboardctrl_state);
	}
	else
	{
		keyboard_state = false;
		TransitionTo(&remotectrl_state);
	}
}
/**
 * @brief 获取视觉模式
 * @parma None
 * @return None
 */
E_pcVisionMode InfantryCTRL_Classdef::Get_pcVisionMode()
{
	return pc_vision_mode;
}
/**
 * @brief 获取运动模式
 * @parma None
 * @return None
 */
E_CapMode InfantryCTRL_Classdef::Get_CapMode()
{
	return cap_mode;
}
/**
 * @brief 获取状态机模式
 * @parma None
 * @return None
 */
E_StateMachine InfantryCTRL_Classdef::Get_StateMachine()
{
	return state_machine;
}
/**
 * @brief 舵轮滞回比较器
 * @parma None
 * @return None
 */
uint8_t InfantryCTRL_Classdef::hysteresis_comparator()
{
	static uint8_t compare_flag = 0;
	if (board_com.rx_pack2.cap_voltage >= 22)
		compare_flag = 1;
	if (board_com.rx_pack2.cap_voltage < 19)
		compare_flag = 0;
	return compare_flag;
}
/**
 * @brief 平衡步卡盲道判断
 * @parma None
 * @return None
 */
bool InfantryCTRL_Classdef::stuck_judge()
{
	static int stuck_judge_cnt;
	static float last_yawAngle;
	if ((abs(chassisCTRL.chassis_yawAngle.Error) > 45 && !rotation_state) || (rotation_state && abs(last_yawAngle - chassisCTRL.chassis_yaw_current) / 10.0f * 1000 < 300))
		stuck_judge_cnt++;
	else
		stuck_judge_cnt = 0;
	last_yawAngle = chassisCTRL.chassis_yaw_current;
	/*1s底盘不动，判断底盘卡住*/
	if (stuck_judge_cnt > 80) // 指示灯任务执行周期10ms
	{
		return true;
	}
	else
		return false;
}

/**
 * @brief 状态清零函数
 * @parma None
 * @return None
 */
void InfantryCTRL_Classdef::clear_state()
{
	fri_state = false;
	laser_state = false;
	turnplate_state = false;
	bulletBay_state = false;
	rotation_state = false;
	cap_mode = NORMAL;
	vision_mode_flag1 = 1;
	vision_mode_flag2 = 1;
	pc_vision_mode = ROTATION_V;
	turn90degrees = false;
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB**************************/
