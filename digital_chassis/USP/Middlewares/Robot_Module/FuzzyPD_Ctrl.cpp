/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file 
  * @author  
  * @brief   
  * @date    
  * @version
  * @par Change Log：
  * <table>
  * <tr><th>Date        <th>Version  <th>Author         <th>Description
  * <tr><td>
  * </table>
  *
  ============================================================================== 
                            How to use this library
  ==============================================================================
    @note
      -# 构造7*7模糊规则表（行为E，列为CE），并调用Load_FuzzyRule加载
					- 对于不需要模糊控制的PID参数，加载时直接传入NULL即可
		  
			-# 调用Fuzzy_Adjust，传入当前误差和误差变化率
			-# 访问Kp_out，Ki_out和Kd_out获取模糊控制器输出的PID参数

    @see
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
/* Includes ------------------------------------------------------------------*/
#include "FuzzyPD_Ctrl.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
/**
* @brief 模糊控制
* @parma 
* @return 
*/
void FuzzyPID_Classdef::Fuzzy_Adjust(float _e, float _ce)
{
	/* 首先进行输入变量模糊化 */
	Quantization(_e, _ce);
	
	/* 计算输入模糊变量的隶属度 */
	Get_GradMembership();
	
	/* 查找模糊规则表，得输出模糊量在各个隶属度下的权重 */
	Get_OutputGrad();
	
	/* 重心法求解输出模糊变量值 */
	Get_FuzzyOut();
	
	/* 反解输出模糊变量，得PID参数输出 */
	Inv_Quantization();
}

/**
* @brief 加载模糊规则表
* @parma 
* @return 
*/
void FuzzyPID_Classdef::Load_FuzzyRule(int8_t (*_kp)[7], int8_t (*_ki)[7], int8_t (*_kd)[7])
{
	kp_rule = _kp;
	ki_rule = _ki;
	kd_rule = _kd;

}

/**
* @brief 输入物理量模糊化，得到输入模糊变量
* @parma 
* @return 
*/
void FuzzyPID_Classdef::Quantization(float _e_input, float _ce_input)
{
	E = 12.0f / (params.e_limit[FUZZY_MAX] - params.e_limit[FUZZY_MIN]) * _e_input;
	CE = 12.0f / (params.ce_limit[FUZZY_MAX] - params.ce_limit[FUZZY_MIN]) * _ce_input;
}

/* 获取输入模糊变量的隶属度，使用timf作隶属度函数 */
void FuzzyPID_Classdef::Get_GradMembership()
{
	/* 计算E的隶属度 */
	/* 若E在论域内，则使用timf作为隶属度函数计算隶属度 */
	if(E > E_membership_val[0] && E < E_membership_val[6])
	{
		for(uint8_t i = 0;i < 6;i++)
		{
			if(E > E_membership_val[i] && E < E_membership_val[i+1])
			{
				/* 计算E归属于左侧的隶属度 */
				E_gradmembership[0] = -(E - E_membership_val[i+1])/(E_membership_val[i+1] - E_membership_val[i]);
				/* 计算E归属于右侧的隶属度 */
				E_gradmembership[1] = 1 - E_gradmembership[0];
				E_gradindex[0] = i;
				E_gradindex[1] = i+1;
			}
		}
	}
	/* 否则，则计算E归属于其左侧的隶属度函数，且隶属度为1 */
	else
	{
		E_gradmembership[0] = 1;
		E_gradmembership[1] = 0;
		E_gradindex[1] = -1;
		
		if(E < E_membership_val[0])
		{
			E_gradindex[0] = 0;
		}
		else
		{
			E_gradindex[0] = 6;
		}
	}
	
	/* 若CE在论域内，则使用timf作为隶属度函数计算隶属度 */
	if(CE > CE_membership_val[0] && CE < CE_membership_val[6])
	{
		for(uint8_t i = 0;i < 6;i++)
		{
			if(CE > CE_membership_val[i] && CE < CE_membership_val[i+1])
			{
				/* 计算CE归属于左侧的隶属度 */
				CE_gradmembership[0] = -(CE - CE_membership_val[i+1])/(CE_membership_val[i+1] - CE_membership_val[i]);
				/* 计算CE归属于右侧的隶属度 */
				CE_gradmembership[1] = 1 - CE_gradmembership[0];
				CE_gradindex[0] = i;
				CE_gradindex[1] = i+1;
			}
		}
	}
	/* 否则，则计算CE归属于其左侧的隶属度函数，且隶属度为1 */
	else
	{
		CE_gradmembership[0] = 1;
		CE_gradmembership[1] = 0;
		CE_gradindex[1] = -1;
		
		if(CE < CE_membership_val[0])
		{
			CE_gradindex[0] = 0;
		}
		else
		{
			CE_gradindex[0] = 6;
		}
	}
}

/**
* @brief 获取输出模糊变量的隶属度
* @parma 
* @return 
*/
void FuzzyPID_Classdef::Get_OutputGrad()
{
	/* 存储查表得到的PID隶属度函数索引 */
	uint8_t index;
	
	/* 每次计算前清除上一次的计算结果 */
	memset(KpgradSums, 0, 7*sizeof(float));
	memset(KigradSums, 0, 7*sizeof(float));
	memset(KdgradSums, 0, 7*sizeof(float));
	
	for(uint8_t i = 0;i < 2;i++)
	{
		/* 若输入模糊量的隶属度出现-1，则跳过本次查表 */
		if(CE_gradindex[i] == -1)
			continue;
		
		for(uint8_t j = 0;j < 2;j++)
		{
			if(E_gradindex[j] == -1)
				continue;
			
			/* 查表获取输出隶属度函数的索引 */
			if(kp_rule != NULL)
			{
				index = kp_rule[CE_gradindex[i]][E_gradindex[j]];
				KpgradSums[index] = KpgradSums[index] + (CE_gradmembership[i] * E_gradmembership[j]);
			}
			
			if(ki_rule != NULL)
			{
				index = ki_rule[CE_gradindex[i]][E_gradindex[j]];
				KigradSums[index] = KigradSums[index] + (CE_gradmembership[i] * E_gradmembership[j]);
			}
			
			if(kd_rule != NULL)
			{
				index = kd_rule[CE_gradindex[i]][E_gradindex[j]];
				KdgradSums[index] = KdgradSums[index] + (CE_gradmembership[i] * E_gradmembership[j]);
			}
		}
	}
}

/**
* @brief 由输出隶属度反解出输出模糊变量值，使用重心法
* @parma 
* @return 
*/
void FuzzyPID_Classdef::Get_FuzzyOut()
{
	KP = 0;
	KI = 0;
	KD = 0;
	
	/* 加权平均，计算输出模糊变量值
		 对于没有传入模糊规则表的模糊变量，其值为0，将在最后一步通过指针非空判断筛选出来！ */
	for(uint8_t i = 0;i < 7;i++)
	{
		KP += KP_membership_val[i] * KpgradSums[i];
		KI += KI_membership_val[i] * KigradSums[i];
		KD += KD_membership_val[i] * KdgradSums[i];
	}
}

/**
* @brief 输出模糊变量反解出输出物理量，即PID参数
* @parma 
* @return 
*/
void FuzzyPID_Classdef::Inv_Quantization()
{
	params.kp_params[FUZZY_SCALE] = params.kp_params[FUZZY_MAX] - params.kp_params[FUZZY_MIN];
	params.ki_params[FUZZY_SCALE] = params.ki_params[FUZZY_MAX] - params.ki_params[FUZZY_MIN];
	params.kd_params[FUZZY_SCALE] = params.kd_params[FUZZY_MAX] - params.kd_params[FUZZY_MIN];
	params.kp_params[FUZZY_BIAS] = params.kp_params[FUZZY_MIN];
	params.ki_params[FUZZY_BIAS] = params.ki_params[FUZZY_MIN];
	params.kd_params[FUZZY_BIAS] = params.kd_params[FUZZY_MIN];
	
	if(kp_rule == NULL)
		Kp_out = 0.0f;
	else
		Kp_out = params.kp_params[FUZZY_BIAS] + params.kp_params[FUZZY_SCALE] / 2.0f * KP;
	
	if(ki_rule == NULL)
		Ki_out = 0.0f;
	else
		Ki_out = params.ki_params[FUZZY_BIAS] + params.ki_params[FUZZY_SCALE] / 2.0f * KI;

	if(kd_rule == NULL)
		Kd_out = 0.0f;
	else
		Kd_out = params.kd_params[FUZZY_BIAS] + params.kd_params[FUZZY_SCALE] / 2.0f * KD;
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
