/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    Infantry_CTRL.h
 * @author  mzh
 * @brief   步兵控制头文件
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
#ifndef _INFANTRY_CTRL_H_
#define _INFANTRY_CTRL_H_

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "board_com.h"
#include "booster.h"
#include "chassisCTRL.h"
#include "gimbal.h"
#include "PCvision.h"
#include "indicator.h"
/* Private macros ------------------------------------------------------------*/
/*步兵类型*/
#define CLASSICAL_INFANTRY 0
#define STEER_INFANTRY 0
#define BALANCE_INFANTRY 1
/*云台初始化宏定义*/
#define PITCH_SCALE_REMOTE 0.175f // 遥控灵敏度
#define YAW_SCALE_REMOTE 0.175f
#define PITCH_SCALE_KEYBOARD 0.4f // 键盘灵敏度
#define YAW_SCALE_KEYBOARD 0.6f

#if CLASSICAL_INFANTRY
#define YAW_OFFSET 5455	  // 云台正方向对应的码盘值，底盘跟随用，新舵轮3420，新麦轮，5455，平衡步7547
#define PITCH_OFFSET 6770 // 新麦轮5450，新舵轮3900，平衡步6770
#endif
#if STEER_INFANTRY
#define YAW_OFFSET 5405
#define PITCH_OFFSET 3900
#endif
#if BALANCE_INFANTRY
#define YAW_OFFSET 5448 // 电容方向->电池方向+4096
#define PITCH_OFFSET 2100
#endif
/*视觉补偿量*/
#define VISION_PITCH_COMPENSATION 0.0f
#define VISION_YAW_COMPENSATION 0.0f
/*视觉发射续弹延时*/
#define VISION_CAN_SHOOT_DELAY 90
/* Private type --------------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim3_ch1_trig;
/* Exported types ------------------------------------------------------------*/
class InfantryCTRL_Classdef;
/**
 * @brief 状态基类
 */
class CState_Base
{
protected:
	InfantryCTRL_Classdef *context;

public:
	/* 将当前状态挂载到上下文中 */
	void Set_Context(InfantryCTRL_Classdef *_context)
	{
		this->context = _context;
	}

	/* 当前状态的处理函数 */
	virtual void Handle_State() = 0;
};
/**
 * @brief 预平衡状态
 */
class PerBalance_State : public CState_Base
{
	virtual void Handle_State();
};
/**
 * @brief 死亡状态
 */
class DeathStranding_State : public CState_Base
{
	virtual void Handle_State();
};
/**
 * @brief 失控状态
 */
class LostCtrl_State : public CState_Base
{
	virtual void Handle_State();
};
/**
 * @brief 遥控控制状态
 */
class RemoteCtrl_State : public CState_Base
{
	virtual void Handle_State();
};
/**
 * @brief 键盘控制状态
 */
class KeyboardCtrl_State : public CState_Base
{
	virtual void Handle_State();
};
/**
 * @brief 视觉控制状态
 */
class PCvisionCtrl_State : public CState_Base
{
	virtual void Handle_State();
};

extern PerBalance_State perbalance_state;
extern DeathStranding_State deathstranding_state;
extern LostCtrl_State lostctrl_state;
extern RemoteCtrl_State remotectrl_state;
extern KeyboardCtrl_State keyboardctrl_state;
extern PCvisionCtrl_State pcvisionctrl_state;

/*电容模式*/
enum E_CapMode
{
	NORMAL = 0,
	UNLIMITED = 1, // 超功率
	ASCENT = 2,	   // 上坡
	LEAP = 3,	   // 飞坡
};

/*视觉模式*/
enum E_pcVisionMode
{
	NORMAL_V = 0,		// 地面目标
	RUNE_V = 1,			// 打符
	AGAINST_RUNE_V = 2, // 反打符
	ROTATION_V = 3,		// 小陀螺
};
/*状态机模式*/
enum E_StateMachine
{
	LOSTCTRL = 0,
	PERBALANCE = 1,
	DEATHSTRANDING = 2,
	REMOTECTRL = 3,
	KEYBOARDCTRL = 4,
};
class InfantryCTRL_Classdef
{
public:
	InfantryCTRL_Classdef() : chassisCTRL(RECTANGULAR),
							  gimbal(YAW_OFFSET, PITCH_OFFSET), // 这里已经在头文件中修改了
							  indicator(&htim3, &hdma_tim3_ch1_trig, TIM_CHANNEL_1)
	{
		/*pid参数初始化*/
		Infantry_Config();
		/* 设置底盘初始状态 */
		this->TransitionTo(&lostctrl_state);
	}

	BoardCom_Classdef board_com;	  // 板间通信类
	Booster_Classdef booster;		  // 小发射类
	ChassisCTRL_Classdef chassisCTRL; // 底盘控制类
	Gimbal_Classdef gimbal;			  // 云台类
	PCvision_Classdef pc_vision;	  // 视觉通信类
	Indicator_Classdef indicator;	  // 指示灯类
	/*传递的标志位*/
	bool reset_state, ui_reset_flag, gg_flag, vision_mode_flag1 = 1, vision_mode_flag2 = 1; // 通用：  遥控保护标志位、刷新ui标志位、手动复位标志位、视觉模式1，视觉模式2
	bool fri_state = 0, laser_state = 0, turnplate_state = 0, bulletBay_state = 0;	// 小发射：摩擦轮、激光、拨盘、弹舱盖标志位
	uint8_t rotation_state = 0, keyboard_state = 0;									// 底盘：  小陀螺、键盘操控标志位
	uint8_t turn90degrees, self_rescue_state, sliding_remake;						// 平衡步：侧身90°标志位，固连自救标志位、滑块复位键

	/*视觉实际发射指令*/
	uint8_t fact_vision_can_shoot = 0;
	int16_t vision_can_shoot_count = VISION_CAN_SHOOT_DELAY; //视觉续弹延时

	/*接口*/
	void Infantry_Config();											// pid参数初始化
	void Status_Update();											// 标志位、参数下发给云台、小发射等
	void Update_StateRequest();										// 更新状态并提交状态切换请求
	void Adjust();													// 云台、小发射等对传入的数据进行计算
	void Actuate();													// 控制量下发到电机
	void Update_Data(CAN_COB CAN_RxMsg);							// 更新电机、板间通信数据
	uint8_t LogicJudge(uint8_t state, uint8_t ctrl, uint8_t *flag); // 键位逻辑判断
	E_pcVisionMode Get_pcVisionMode();								// 获取当前视觉模式
	E_CapMode Get_CapMode();										// 获取当前运动模式
	E_StateMachine Get_StateMachine();								// 获取当前状态机模式
	/*平衡步卡盲道判断*/
	bool stuck_judge();
	/*延时变量*/
	int death_delay_cnt = 0; // 通过判断can是否进入中断来判断死亡的标志位
	/*电容电压*/
	float cap_voltage = 0;
private:
	/*声明友元类*/
	friend PerBalance_State;
	friend DeathStranding_State;
	friend LostCtrl_State;
	friend RemoteCtrl_State;
	friend KeyboardCtrl_State;
	friend PCvisionCtrl_State;

	CState_Base *current_state;
	/* 提交切换需求，切换到下一个状态 */
	void TransitionTo(CState_Base *_state)
	{
		this->current_state = _state;
		this->current_state->Set_Context(this);
	}
	/*判断键鼠或遥控模式*/
	void Judge_CTRLmode();

	/*状态参数*/
	E_CapMode cap_mode;
	E_pcVisionMode pc_vision_mode = ROTATION_V;
	E_StateMachine state_machine;
	/*下发到云台、底盘控制等对象的参数*/
	float pitch_data, yaw_data; // 发给云台
	float x_data, y_data;		// 发给底盘控制
	float x_back_data, y_back_data;
	/*平衡步倒地自救相关参数*/
	int perbalance_delay_cnt = 1500; // 倒地自救延时
	uint8_t enable_cmd;				 // 底盘使能标志位
	/*舵轮电容电压滞回比较器*/
	uint8_t hysteresis_comparator();
	/*状态清零函数*/
	void clear_state();
};
/* Exported function declarations --------------------------------------------*/
#endif

#ifdef __cplusplus
extern "C"
{
#endif
	/* Exported macros -----------------------------------------------------------*/
	/* Exported types ------------------------------------------------------------*/
	/* Exported function declarations --------------------------------------------*/
}
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
