/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    gimbal.h
  * @author  mzh
  * @brief   云台代码头文件
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have any 
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#ifndef _GIMBAL_H_
#define _GIMBAL_H_

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "SRML.h"
#include "srml_std_lib.h"
/* Private macros ------------------------------------------------------------*/
#define ID_PITCH			(1)
#define ID_YAW 				(2)
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/*pitch、yaw反馈值类型*/
typedef enum E_PitchYaw_FeedbackBy{
	ENCODER = 0U,
	GYRO,
}E_PitchYaw_FeedbackBy;
/*pitch、yaw pid各环,操作手模式下及自瞄下*/
enum E_PitchYawPidType
{
	PITCH_SPEED_1 			= 0,
	PITCH_ANGLE_1				= 1,
	YAW_SPEED_1  				= 2,
	YAW_ANGLE_1					= 3,
	PITCH_SPEED_2 			= 4,
	PITCH_ANGLE_2				= 5,
	YAW_SPEED_2  				= 6,
	YAW_ANGLE_2					= 7,
	PITCH_SPEED_3 			= 8,
	PITCH_ANGLE_3				= 9,
	YAW_SPEED_3  				= 10,
	YAW_ANGLE_3					= 11,
};
/*pid结构体*/
struct S_PidPara
{
	float kp;
	float ki;
	float kd;
	float I_Term_Max;
	float PI_Term_Max;
	float Out_Max;
	float I_SeparThresh;
};

class Gimbal_Classdef
{
	public:
		Gimbal_Classdef(int16_t _yaw_offset/*底盘跟随目标角*/,int16_t _pitch_offset)
		{
			/*底盘跟随角度初始化*/
			yawMotor.setEncoderOffset(_yaw_offset);
			pitchMotor.setEncoderOffset(_pitch_offset);
		}
		/*创建电机对象*/
		Motor_GM6020 pitchMotor = Motor_GM6020(ID_PITCH);
		Motor_GM6020 yawMotor	  = Motor_GM6020(ID_YAW); 
		/*创建pid对象*/
		myPID pitch_speedloop;
		myPID pitch_angleloop;
		myPID yaw_speedloop;
		myPID yaw_angleloop;
		float pitch_anglekd;
		float yaw_anglekd;
		/*接口*/
		void Status_Update(float*_pitchData,float*_yawData,bool*_resetState);
		void MPUdata_Update(mpu_rec_s*p);																	//pitch yaw角度更新，包括初始角度偏置
		void Adjust();
		void Pack_CAN(int8_t _CAN_number,Motor_CAN_COB*_CANX_pack);				//打包函数	
		/*获取当前值*/
		float Get_PitchTarget();																					//pitch目标值
		float Get_PitchCurrent();																					//pitch陀螺仪当前值
		float Get_YawTotal();																							//云台yaw当前值
		float Get_YawMotorAngle();																				//yaw码盘电机值
		float Get_YawTarget();																						//yaw目标值
		float Get_Angular_Velocity_Pitch();																//pitch角速度
		float Get_Angular_Velocity_Yaw();																	//yaw角速度
		/*设置目标值*/	
		void Set_PitchTarget(float _pitch_target);
		void Set_YawTarget(float _yaw_target);									
		void Set_FeedbackSource(E_PitchYaw_FeedbackBy);										//设置反馈来源
		void Reset_YawUpdateFlag();																				//平衡步用，陀螺仪计算角度复位
		/*pid相关*/
		void PidParaInit(E_PitchYawPidType _type, float _kp, float _ki, float _kd, float _ki_max, float _pi_max,float _out_max, float _I_SeparThresh);	
																																			//pid参数初始化
		void LoadPidPara(myPID *_pidloop,S_PidPara _pid_para);						//加载pid参数进pid对象
		void Switch2VisionPid(uint8_t _vision_pid_mode);									//切换到视觉pid
		S_PidPara pid_para[12];																						//pid各环参数
		
		float pitch_target,yaw_target;

	private:		
		/*pitch、yaw相关变量*/
	  bool yawUpdate_is_init;																						//yaw复位标志位,平衡步倒地自救用
		bool gimbal_resetState;																						//云台复位标志位
		float pitch_scale,yaw_scale;																			//pitch、yaw灵敏度
		float total_yaw;

		E_PitchYaw_FeedbackBy feedback_by;																//控制pitch yaw的数据来源
		/*陀螺仪相关变量*/
		float angular_velocity_yaw,angular_velocity_pitch,current_pitch,current_yaw;	//陀螺仪传入
			
		/*pid计算函数*/
		void gimbal_pid_calculate();
		/*yaw角度计算函数*/
		void yaw_calculate();
		/*云台复位函数*/
		void gimbal_reset();		
};
/* Exported function declarations --------------------------------------------*/
#endif

#ifdef __cplusplus
extern "C"{
#endif
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/	
}
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
