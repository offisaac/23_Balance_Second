/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    
  * @author 
  * @brief   
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
#ifndef __FUZZYPD_CTRL_H_
#define __FUZZYPD_CTRL_H_

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/
//#define NB	0
//#define NM	1
//#define NS	2
//#define ZE	3
//#define PS	4
//#define PM	5
//#define PB	6
#define ZE	0
#define PS	1
#define PB	2

/* Private type --------------------------------------------------------------*/
enum{
	FUZZY_MIN = 0U,
	FUZZY_MAX	= 1U,
	FUZZY_BIAS = 2U,
	FUZZY_SCALE = 3U
};

typedef struct FuzzyParams_Typedef{
	float e_limit[2];
	float ce_limit[2];
	float kp_params[4];
	float ki_params[4];
	float kd_params[4];
	
}FuzzyParams_Typedef;

/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class FuzzyPID_Classdef{
	private:
	/* 各个模糊量的隶属度函数中值
		 隶属度函数采用timf，共七个 */
	const int16_t E_membership_val[7]  = {-6, -4, -2, 0, 2, 4, 6};
	const int16_t CE_membership_val[7] = {-6, -4, -2, 0, 2, 4, 6};
	const int16_t KP_membership_val[7] = {0, 1, 2, 3, 4, 5, 6};
	const int16_t KI_membership_val[7] = {0, 1, 2, 3, 4, 5, 6};
	const int16_t KD_membership_val[7] = {0, 1, 2, 3, 4, 5, 6};
	
	/* PID模糊规则表指针，指针形式便于使用同个类创建多个模糊控制器 */
	int8_t (*kp_rule)[7] = NULL;
	int8_t (*ki_rule)[7] = NULL;
	int8_t (*kd_rule)[7] = NULL;
	

	
	/* 输入物理量模糊化，得到输入模糊变量 */
	void Quantization(float _e_input, float _ce_input);
	
	/* 获取输入模糊变量的隶属度，使用timf作隶属度函数 */
	void Get_GradMembership();
	
	/* 获取输出模糊变量的隶属度 */
	void Get_OutputGrad();
	
	/* 由输出隶属度反解出输出模糊变量值，使用重心法 */
	void Get_FuzzyOut();
	
	/* 输出模糊变量反解出输出物理量，即PID参数 */
	void Inv_Quantization();
	
	public:
	/* 模糊输出量在各个输出隶属函数上的隶属度
		 输出隶属函数采用timf，共七个 */
	float KpgradSums[7] = {0};
	float KigradSums[7] = {0};
	float KdgradSums[7] = {0};

	/* 输入模糊量映射 */
	float E = 0.0f;																				//误差的模糊变量
	float CE = 0.0f;																			//误差变化率的模糊变量
	float E_gradmembership[2] = {0};											//误差模糊量映射后对应的隶属度
	float CE_gradmembership[2] = {0};								
	int8_t E_gradindex[2] = {0};													//误差模糊量映射后对应的隶属函数索引，即属于哪个隶属函数
	int8_t CE_gradindex[2] = {0};	
	
	/* 输出模糊量 */
	float KP = 0.0f;
	float KI = 0.0f;
	float KD = 0.0f;
	
	/* 模糊控制器最终输出，即PID参数 */
	float Kp_out = 0.0f;
	float Ki_out = 0.0f;
	float Kd_out = 0.0f;

		/* 模糊控制器参数 */
	FuzzyParams_Typedef params;
	FuzzyPID_Classdef(float _e_min, float _e_max, float _ce_min, float _ce_max, \
										float _kp_min, float _kp_max, float _ki_min, float _ki_max, float _kd_min, float _kd_max){
		this->params.e_limit[FUZZY_MIN] = _e_min;
		this->params.e_limit[FUZZY_MAX] = _e_max;
		this->params.ce_limit[FUZZY_MIN] = _ce_min;
		this->params.ce_limit[FUZZY_MAX] = _ce_max;
		
		this->params.kp_params[FUZZY_MIN] = _kp_min;
		this->params.kp_params[FUZZY_MAX] = _kp_max;
		this->params.ki_params[FUZZY_MIN] = _ki_min;
		this->params.ki_params[FUZZY_MAX] = _ki_max;
		this->params.kd_params[FUZZY_MIN] = _kd_min;
		this->params.kd_params[FUZZY_MAX] = _kd_max;
		
	}
	
	/* 加载模糊规则表 */
	void Load_FuzzyRule(int8_t (*_kp)[7], int8_t (*_ki)[7], int8_t (*_kd)[7]);

	/* 模糊控制 */
	void Fuzzy_Adjust(float _e, float _ce);
	
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
