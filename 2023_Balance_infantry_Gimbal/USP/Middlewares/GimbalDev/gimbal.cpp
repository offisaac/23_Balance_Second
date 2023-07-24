/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file gimbal.cpp
 * @author mzh
 * @brief 云台代码
 * @date 2022-02-11
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
#include "gimbal.h"
#include "math.h"
#include "DiffCalculator.h"
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
/* function prototypes -------------------------------------------------------*/
/**
 * @brief 函数接口，获取状态标志位，及遥感量
 * @param None
 * @retval None
 */
void Gimbal_Classdef::Status_Update(float *_pitchData, float *_yawData, bool *_resetState)
{
	gimbal_resetState = *_resetState;
	pitch_target += *_pitchData;
	yaw_target += *_yawData;
}
/**
 * @brief 函数接口，获取陀螺仪数据
 * @param None
 * @retval None
 */
void Gimbal_Classdef::MPUdata_Update(mpu_rec_s *p)
{
	angular_velocity_yaw = p->gyro[2];
	angular_velocity_pitch = p->gyro[1];
	current_yaw = p->yaw;
	current_pitch = p->roll;
}
/**
 * @brief 函数接口，处理云台数据
 * @param None
 * @retval None
 */
void Gimbal_Classdef::Adjust()
{
	yaw_calculate();
	if (gimbal_resetState)
	{
		gimbal_reset();
	}
	else
	{
		gimbal_pid_calculate();
	}
}
/**
 * @brief 打包函数
 * @param None
 * @retval None
 */
void Gimbal_Classdef::Pack_CAN(int8_t _CAN_numb, Motor_CAN_COB *_CANX_pack)
{
	switch (_CAN_numb)
	{
	case 1:
		(*_CANX_pack) = MotorMsgPack<Motor_GM6020>(*_CANX_pack, pitchMotor);
		break;
	case 2:
		(*_CANX_pack) = MotorMsgPack<Motor_GM6020>(*_CANX_pack, yawMotor);
		break;
	default:
		break;
	}
}
/**
 * @brief 获取当前值
 * @param None
 * @retval Pitch,Yaw当前值
 */
float Gimbal_Classdef::Get_PitchTarget()
{
	return pitch_target;
}

float Gimbal_Classdef::Get_PitchCurrent()
{
	return current_pitch;
}
float Gimbal_Classdef::Get_YawTotal()
{
	return total_yaw;
}
float Gimbal_Classdef::Get_YawTarget()
{
	return yaw_target;
}
float Gimbal_Classdef::Get_YawMotorAngle()
{
	return yawMotor.getAngle();
}
float Gimbal_Classdef::Get_Angular_Velocity_Pitch()
{
	return angular_velocity_pitch;
}
float Gimbal_Classdef::Get_Angular_Velocity_Yaw()
{
	return angular_velocity_yaw;
}
/**
 * @brief 设置目标值
 * @param None
 * @retval None
 */
void Gimbal_Classdef::Set_PitchTarget(float _pitch_target)
{
	pitch_target = _pitch_target;
}
void Gimbal_Classdef::Set_YawTarget(float _yaw_target)
{
	yaw_target = _yaw_target;
}
void Gimbal_Classdef::Set_FeedbackSource(E_PitchYaw_FeedbackBy _feedback_by)
{
	if (_feedback_by != feedback_by)
	{
		feedback_by = _feedback_by;
	}
}
void Gimbal_Classdef::Reset_YawUpdateFlag()
{
	yawUpdate_is_init = false;
}
/**
 * @brief pid参数初始化
 * @param None
 * @retval None
 */
void Gimbal_Classdef::PidParaInit(E_PitchYawPidType _type, float _kp, float _ki, float _kd, float _ki_max, float _pi_max, float _out_max, float _I_SeparThresh)
{
	pid_para[_type].kp = _kp;
	pid_para[_type].ki = _ki;
	pid_para[_type].kd = _kd;
	pid_para[_type].I_Term_Max = _ki_max;
	pid_para[_type].PI_Term_Max = _pi_max;
	pid_para[_type].Out_Max = _out_max;
	pid_para[_type].I_SeparThresh = _I_SeparThresh;
}
/**
 * @brief pid对象加载pid参数
 * @param None
 * @retval None
 */
void Gimbal_Classdef::LoadPidPara(myPID *_pidloop, S_PidPara _pid_para)
{
	_pidloop->SetPIDParam(_pid_para.kp, _pid_para.ki, _pid_para.kd, _pid_para.I_Term_Max, _pid_para.PI_Term_Max, _pid_para.Out_Max);
	_pidloop->I_SeparThresh = _pid_para.I_SeparThresh;
}
/**
 * @brief 切换到视觉pid参数
 * @param None
 * @retval None
 */
void Gimbal_Classdef::Switch2VisionPid(uint8_t _vision_pid_mode)
{
	switch (_vision_pid_mode)
	{
	case 1:
	{
		/*载入地面目标pid参数*/
		LoadPidPara(&pitch_speedloop, pid_para[PITCH_SPEED_1]);
		LoadPidPara(&pitch_angleloop, pid_para[PITCH_ANGLE_1]);
		LoadPidPara(&yaw_speedloop, pid_para[YAW_SPEED_1]);
		LoadPidPara(&yaw_angleloop, pid_para[YAW_ANGLE_1]);
		break;
	}
	case 2:
	{
		/*载入神符pid参数*/
		LoadPidPara(&pitch_speedloop, pid_para[PITCH_SPEED_2]);
		LoadPidPara(&pitch_angleloop, pid_para[PITCH_ANGLE_2]);
		LoadPidPara(&yaw_speedloop, pid_para[YAW_SPEED_2]);
		LoadPidPara(&yaw_angleloop, pid_para[YAW_ANGLE_2]);
		break;
	}
	case 3:
	{
		/*载入哨兵pid参数*/
		LoadPidPara(&pitch_speedloop, pid_para[PITCH_SPEED_3]);
		LoadPidPara(&pitch_angleloop, pid_para[PITCH_ANGLE_3]);
		LoadPidPara(&yaw_speedloop, pid_para[YAW_SPEED_3]);
		LoadPidPara(&yaw_angleloop, pid_para[YAW_ANGLE_3]);
		break;
	}
	}
	/*清除I项积分*/
	//	 pitch_speedloop.I_Term = pitch_angleloop.I_Term = yaw_speedloop.I_Term = yaw_angleloop.I_Term = 0;
}
/**
 * @brief pid计算
 * @param None
 * @retval None
 */
float yaw_out = 0;
float sin_w = 3.1415926f;
float ta_o = 0;		 //三角波幅值
uint16_t ta_f = 1; //频率
int16_t ta_count = 0;
float t = 0;
uint8_t debug_switch = 0;

float m_r = 53.f;
float m_l = 0.0004f;
//速度环
float speed_kp = -150;
float speed_ki = -1000;
float speed_imax = 1000;
float speed_omax = 17000;
void Gimbal_Classdef::gimbal_pid_calculate()
{
	pitch_controller.SetAngleloopParams(25,300);
	pitch_controller.SetSpeedloopParams(150,2000,1000,30000,400);
	pitch_controller.SetCurrentloopParams(0.25,50,3000,30000,120000);
	t = Get_SystemTimer() * 0.000001f;
	/*更新当前值，并考虑切换不同的反馈通路*/
	if (feedback_by == ENCODER)
	{
		pitch_angleloop.Current = -pitchMotor.getAngle();
		pitch_controller.update_current_state(-pitchMotor.getAngle(),angular_velocity_pitch,pitchMotor.givenCurrent,yawMotor.getSpeed());
		/*以最小角度跟随*/
		yaw_controller.update_current_state(int(yawMotor.getAngle()) % 360, angular_velocity_yaw, yawMotor.givenCurrent, yawMotor.getSpeed());
		yaw_angleloop.Current = int(yawMotor.getAngle()) % 360;
		if (yaw_controller.angleLoop.Current > 180)
		{
			yaw_controller.angleLoop.Current -= 360;
		}
		else if (yaw_controller.angleLoop.Current < -180)
		{
			yaw_controller.angleLoop.Current += 360;
		}
		
		if (yaw_angleloop.Current > 180)
		{
			yaw_angleloop.Current -= 360;
		}
		else if (yaw_angleloop.Current < -180)
		{
			yaw_angleloop.Current += 360;
		}
	}
	else
	{
		pitch_angleloop.Current = current_pitch;
		pitch_controller.update_current_state(current_pitch,angular_velocity_pitch,pitchMotor.givenCurrent,yawMotor.getSpeed());
		yaw_controller.update_current_state(total_yaw, angular_velocity_yaw, yawMotor.givenCurrent, yawMotor.getSpeed());
		yaw_angleloop.Current = total_yaw;
	}
	pitch_speedloop.Current = angular_velocity_pitch;

	/*更新目标值*/
	pitch_target = std_lib::constrain(pitch_target, -20.0f, 30.0f);
	pitch_angleloop.Target = pitch_target;
	pitch_controller.update_target_angle(pitch_target);
	yaw_angleloop.Target = yaw_target;
	yaw_controller.update_target_angle(yaw_target);

	/*计算输出值*/
	pitchMotor.Out = pitch_angleloop.Adjust_importDiff(angular_velocity_pitch) + 116.7360f * current_pitch + 668.5741f; // old
	//	pitchMotor.Out = pitch_angleloop.Adjust_importDiff(angular_velocity_pitch) + 51.38340f * current_pitch  - 438.2411f;//new
	//pitchMotor.Out = pitch_controller.adjust() + 116.7360f * current_pitch + 668.5741f;

	//yawMotor.Out = yaw_angleloop.Adjust_importDiff(angular_velocity_yaw);
	yawMotor.Out = yaw_controller.adjust();

	// /*生成三角波*/
	// if(ta_o!=0)
	// {
	// 	ta_count++;
	// }
	// ta_count = std_lib::constrain(ta_count,(int16_t)0,(int16_t)(1000/ta_f));
	// if(ta_count>=(int16_t)(1000/ta_f))
	// {
	// 	ta_count = 0;
	// }
	// if(ta_count >= 0 && ta_count < (int16_t)(0.25*1000/ta_f))
	// {
	// 	yaw_out += ta_o/(float)(250/ta_f);
	// }
	// else if(ta_count >= (int16_t)(0.25*1000/ta_f) && ta_count < (int16_t)(0.75*1000/ta_f))
	// {
	// 	yaw_out -= ta_o/(float)(250/ta_f);
	// }
	// else
	// {
	// 	yaw_out += ta_o/(float)(250/ta_f);
	// }
	// if(ta_o == 0)
	// {
	// 	yaw_out = 0;
	// 	ta_count = 0;
	// }

	/*生成正弦波*/
	// yaw_out = 200 + 50 * sinf(3 * sin_w * t) + 100 * sinf(sin_w * t);
	// yaw_out = 50 * sinf(sin_w * t);
	if(debug_switch)
	{
		pitch_speedloop.Target = yaw_out;
	}
	else
	{
		pitch_speedloop.Target = 0;
	}
	pitch_speedloop.Current = angular_velocity_pitch;
	pitch_speedloop.SetPIDParam(speed_kp,speed_ki,0,speed_imax,speed_omax,speed_omax);
	pitch_currentloop.Target = pitch_speedloop.Adjust() + 116.2253f * current_pitch - 994.4664f;
	pitch_currentloop.Current = pitchMotor.givenCurrent;
	pitch_currentloop.SetPIDParam(0.25,50,0,3000,30000,30000);
	//pitchMotor.Out = pitch_currentloop.Adjust() + (pitch_currentloop.Target + m_r * pitchMotor.getSpeed()) / (0.75f - m_l * pitchMotor.getSpeed());
}
/**
 * @brief yaw角度计算
 * @param None
 * @retval None
 */
void Gimbal_Classdef::yaw_calculate()
{
	static float last_yaw, yaw_offset;
	static int16_t yaw_cnt = 0;
	/* 若使用编码器赋值，则不使用陀螺仪计算pitch yaw角 */
	if ((feedback_by == ENCODER) && !(yawUpdate_is_init))
		return;
	/*云台复位之后交给陀螺仪控制获取offset*/
	if (yawUpdate_is_init)
	{
		if (current_yaw - last_yaw > 180)
			yaw_cnt--;
		else if (current_yaw - last_yaw < -180)
			yaw_cnt++;
	}
	else
	{
		yaw_cnt = 0;
		yaw_offset = current_yaw; //+(Missile_yaw_6020.getEncoder()-1925) / 8192.0f *360;
		yawUpdate_is_init = true;
	}
	last_yaw = current_yaw;
	total_yaw = yaw_cnt * 360.0f + current_yaw - yaw_offset;
}
/**
 * @brief 云台复位
 * @param None
 * @retval None
 */
void Gimbal_Classdef::gimbal_reset()
{
	/*目标值复位*/
	yaw_target = total_yaw;
	pitch_target = 0;
	/*输出置零*/
	pitchMotor.Out = yawMotor.Out = 0;
	pitch_speedloop.I_Term = pitch_angleloop.I_Term = yaw_speedloop.I_Term = yaw_angleloop.I_Term = 0;
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB**************************/
