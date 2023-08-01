#include "Balance_Chassis.h"

float debug_A;
float debug_B;
float debug_C;
float debug_D;
float debug_E;
float debug_F;
float debug_G;
float debug_H;
float debug_I;

//状态机逻辑实现变量
uint16_t Land_Delay = 200; //倒地自救延时
bool is_reset = 0;         //底盘重置

Lost_Ctrl_State lostctrl_state;
Pre_Balance_State prebalance_state;
Balance_State balance_state;

void Lost_Ctrl_State::State_Handler()
{
    Land_Delay = 200; //计数器重置
    is_reset = 1;
    context->Reset_Adjust();
}

//预平衡状态只在程序重新开始的时候生效
void Pre_Balance_State::State_Handler()
{
    //    static uint8_t rescue_complete = 0; //自救完成标志位

    //    is_reset = 0;

    //    if (Land_Delay < 100) //云台归中 300ms
    //    {
    //        rescue_complete = 1;
    //    }

    //    if (!Land_Delay) //延时结束
    //    {
    //        rescue_complete = 0; //自救标志位归位
    //    }
    //    else
    //    {
    //        (Land_Delay == 0) ? (Land_Delay = 0) : (Land_Delay--); //防止溢出
    //    }

    //    if (rescue_complete) //救援完成标志位发送到云台
    //    {
    //        context->Board_Com.gimbal_rx_pack2.chassis_flags |= 0x0001 << 1;
    //    }
    //    else
    //    {
    //        context->Board_Com.gimbal_rx_pack2.chassis_flags &= 0x0000 << 1;
    //    }

    // vTaskDelay(400);

    if (context->gimbal_data.enable_cmd)
    {
        context->Status_Switching(&balance_state); //底盘使能，进入平衡状态
    }
}

void Balance_State::State_Handler()
{
    is_reset = 0;
    /* 底盘电机控制 */
    context->Chassis_Ctrl_Cal();
    /*进入软件复位*/
    if (context->gimbal_data.gg_flag)
    {
        __set_FAULTMASK(1); //关闭所有中断
        NVIC_SystemReset();
    }
}

/**
 * @brief  构造函数
 * @note
 * @param
 * @return
 * @retval  None
 */
#if DIGITAL_POWER
Balance_Infantry_Classdef::Balance_Infantry_Classdef() : Power_Ctrl(2, REAL_POWER_LOOP, __ENABLE, 2),
                                                         LPMS(1, 1)
{
    speed_scale = 0.25;
    max_wheel_speed = 9000.0f;
    max_launch_speed = 9000.0f;
    max_wheel_output = 16384.0f;
    current_state = &lostctrl_state;
    /*9025抽象类初始化*/
    absWheelMotor[RIGHT].Polarity = -1;
    absWheelMotor[LEFT].Polarity = 1;

    absWheelMotor[RIGHT].bindMotor(&wheel_motor[RIGHT]);
    absWheelMotor[LEFT].bindMotor(&wheel_motor[LEFT]);

    /*陀螺仪抽象类初始化*/
    absLpms.bindIMU(&LPMS);
    absLpms.bindAccCoordinate(abstractIMU::imuWorldAccZ, abstractIMU::imuWorldAccY, abstractIMU::imuWorldAccX);
    absLpms.bindEulerCoordinate(abstractIMU::imuWorldRoll, abstractIMU::imuWorldYaw, abstractIMU::imuWorldPitch);

    absLpms.accPolarity.x = 1;
    absLpms.accPolarity.y = 1;
    absLpms.accPolarity.z = 1;

    absLpms.eularPolarity.pitch = 1;
    absLpms.eularPolarity.roll = 1;
    absLpms.eularPolarity.yaw = 1;

    absLpms.eularBaseData.pitch = 0.f * ratio_degree2rad;
    absLpms.eularBaseData.roll = 0.f * ratio_degree2rad;
    absLpms.eularBaseData.yaw = 0;
}
#else
Balance_Infantry_Classdef::Balance_Infantry_Classdef() : Source_Manage(BAT),
                                                         Power_Ctrl(2, REAL_POWER_LOOP, __ENABLE, 2),
                                                         LPMS(1, 1)
{
    speed_scale = 0.25;
    max_wheel_speed = 9000.0f;
    max_launch_speed = 9000.0f;
    max_wheel_output = 16384.0f;
    current_state = &lostctrl_state;
    /*功率控制初始化*/
}
#endif

/**
 * @brief  队列配置函数
 * @note
 * @param   电机发送队列，板间通信队列
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Load_Chassis_Queue(QueueHandle_t *_motor_queue, QueueHandle_t *_board_queue)
{
    board_queue = _board_queue;
    wheel_motor[LEFT].init(*_motor_queue);
    wheel_motor[RIGHT].init(*_motor_queue);

    wheel_motor[LEFT].writePidToRAM(50, 50, 75, 25, 125, 25);
    vTaskDelay(5);
    wheel_motor[RIGHT].writePidToRAM(50, 50, 75, 25, 125, 25);
}

///**
// * @brief  加载底盘控制器
// * @note
// * @param   当前位姿，目标位姿，当前速度，目标速度
// * @return
// * @retval  None
// */
// void Balance_Infantry_Classdef::Load_Balance_Controller(float (*pFunc)(const float pos_current, const float pos_target, const float speed_current, const float speed_target))
//{
//    balance_controller = pFunc;
//}

/**
 * @brief  板间通信接收数据
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Gimbal_Data_Update(CAN_COB *CAN_RxMsg)
{
    if (CAN_RxMsg->ID == TOCHASSIS_PACK1_ID)
    {
        memcpy((uint8_t *)&gimbal_data, CAN_RxMsg->Data, CAN_RxMsg->DLC);
        auto_mode = gimbal_data.vision_mode_flag1 * 2 + gimbal_data.vision_mode_flag2;
    }
    else if (CAN_RxMsg->ID == TOCHASSIS_PACK2_ID)
    {
        memcpy((uint8_t *)&gimbal_data.chassis_rotation_angle, CAN_RxMsg->Data, CAN_RxMsg->DLC);
    }
    else
    {
    }
}

/**
 * @brief  电机数据更新
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::CAN_Motor_Update(CAN_COB *CAN_RxMsg)
{
    if (wheel_motor[LEFT].CheckID(CAN_RxMsg->ID))
    {
        wheel_motor[LEFT].update(CAN_RxMsg->Data);
        motorLinkCount[LEFT] = 0;
    }
    else if (wheel_motor[RIGHT].CheckID(CAN_RxMsg->ID))
    {
        wheel_motor[RIGHT].update(CAN_RxMsg->Data);
        motorLinkCount[RIGHT] = 0;
    }
}

/**
 * @brief  陀螺仪数据更新
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::UART_Gyro_Update(uint8_t *_rx_msg)
{
    LPMS.LPMS_BE2_Get_Data(_rx_msg);
}

/**
 * @brief  状态切换判断函数
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Judge_State()
{
    if (!gimbal_data.remote_ctrl_state)
    {
        Status_Switching(&lostctrl_state);
    }
    else if (current_state == &lostctrl_state)
    {
        Status_Switching(&prebalance_state);
    }
    else
    {
    }

    balance_controller.Update_Flags(gimbal_data.turn90degrees, gimbal_data.rotation_state, is_reset, gimbal_data.leap_state, gimbal_data.unlimited_state, gimbal_data.ascent_state); //更新控制器标志位

    current_state->State_Handler();
}

/**
 * @brief  更新底盘运动目标值
 * @note
 * @param
 * @return
 * @retval  None
 */
float debug_angle = 100;
void Balance_Infantry_Classdef::Update_Target(float _y_speed, float _z_speed, float _x_speed)
{
    float rotation_speed = (360.f + rotation_scale * 360.f) * DEGREE_TO_RAD;
    // float rotation_speed = 360.f;
    // if (gimbal_data.unlimited_state)
    // {
    //     rotation_speed = 720.f;
    // }

    target_rotation_r = sqrtf(powf(_y_speed, 2) + powf(_x_speed, 2)); //计算平移矢量模长
    if (_y_speed == 0 && _x_speed > 0)
    {
        target_rotation_angle = 90. / 180. * PI; //计算平移矢量角度
    }
    else if (_y_speed == 0 && _x_speed < 0)
    {
        target_rotation_angle = -90. / 180. * PI; //计算平移矢量角度
    }
    else if (_y_speed < 0 && _x_speed > 0)
    {
        target_rotation_angle = atanf(_x_speed / _y_speed) + PI;
    }
    else if (_y_speed < 0 && _x_speed < 0)
    {
        target_rotation_angle = atanf(_x_speed / _y_speed) - PI;
    }
    else if (_y_speed == 0 && _x_speed == 0)
    {
        target_rotation_angle = 0;
    }
    else if (_x_speed == 0 && _y_speed > 0)
    {
        target_rotation_angle = 0;
    }
    else if (_x_speed == 0 && _y_speed < 0)
    {
        target_rotation_angle = -PI;
    }
    else
    {
        target_rotation_angle = atanf(_x_speed / (float)_y_speed); //计算平移矢量角度
    }

    if (gimbal_data.rotation_state)
    {
        target_rotation_angle -= (0.25f * PI * (-balance_controller.current_angularSpeed.yaw - 2 * PI) / (2 * PI) + 0.5f * PI); //根据小陀螺转速进行速度向量相位补偿
    }
    angle_error = rotation_chassis_angle - target_rotation_angle; //得出速度向量与底盘坐标系夹角

    fact_target_speed = target_rotation_r * cosf(angle_error);

    if (gimbal_data.rotation_state)
    {
        balance_controller.Update_Target_LinearSpeed(0, std_lib::constrain(fact_target_speed, -1.0f, 1.0f) * speed_scale * rotation_move_gain, 0);
        balance_controller.Update_Target_AngularSpeed(std_lib::constrain(_z_speed * 6 * PI, -rotation_speed, rotation_speed), 0, 0);
        balance_controller.down_slope_flag = false;
    }
    else
    {
        if (gimbal_data.turn90degrees)
        {
            balance_controller.Update_Target_LinearSpeed(0, std_lib::constrain(fact_target_speed, -0.5f, 0.5f) * speed_scale, 0);
            balance_controller.Update_Target_AngularSpeed(std_lib::constrain(_z_speed, -1.0f, 1.0f) * 6 * PI, 0, 0);
            balance_controller.down_slope_flag = false;
        }
        else if (gimbal_data.ascent_state)
        {
            balance_controller.Update_Target_LinearSpeed(0, std_lib::constrain(fact_target_speed, -0.5f, 0.5f) * speed_scale, 0);
            balance_controller.Update_Target_AngularSpeed(std_lib::constrain(_z_speed, -1.0f, 1.0f) * 6 * PI, 0, 0);
            balance_controller.down_slope_flag = true;
        }
        else
        {
            balance_controller.Update_Target_LinearSpeed(0, std_lib::constrain(fact_target_speed, -1.0f, 1.0f) * speed_scale, 0);
            balance_controller.Update_Target_AngularSpeed(std_lib::constrain(_z_speed, -1.0f, 1.0f) * 6 * PI, 0, 0);
            balance_controller.down_slope_flag = false;
        }
    }
}

/**
 * @brief  更新底盘当前位姿
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Update_Current_Pos(float _yaw, float _pitch, float _chassis_angle)
{
    balance_controller.Update_Current_Pos(_yaw, _pitch, 0);
    rotation_chassis_angle = _chassis_angle / 180. * PI;
}

/**
 * @brief  更新底盘当前速度
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Update_Current_Speed(float _y, float _yaw, float _pitch)
{
    static float distance = 0.0f;
    static uint32_t time_gap = 0;
    static uint32_t current_time = 0;
    static uint32_t last_time = 0;
    current_time = Get_SystemTimer();
    time_gap = current_time - last_time;
    static MeanFilter<20> speed_mf;
    distance += speed_mf.f(_y + _pitch * WHEEL_R) * CTRL_INTERAL;
    balance_controller.Update_Current_LinearSpeed(0, speed_mf.f(_y + _pitch * WHEEL_R), 0); //当前线速度需要减去因车体旋转造成的误差
    balance_controller.Update_Current_AngularSpeed(_yaw, _pitch, 0);
    if (time_gap && current_time && last_time)
    {
        balance_controller.Update_Current_Location(0, distance, 0); //更新走过的距离
    }
    else
    {
    }
    last_time = time_gap;
}

/**
 * @brief  更新底盘当前线性加速度
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Update_Current_Acc(float _x, float _y, float _z)
{
    static MeanFilter<20> xAccF;
    static MeanFilter<20> yAccF;
    static MeanFilter<20> zAccF;
    balance_controller.Update_Current_LinearAcc(xAccF.f(_x), yAccF.f(_y), zAccF.f(_z));
}

/**
 * @brief  更新电机输出电流
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Update_Motor_Current(float _current)
{
    balance_controller.motor_current = _current;
}

/**
 * @brief  更新滑块位置和转速
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Update_Slider_Params(float _s[2], float _sspeed[2])
{
    for (int i = 0; i < 2; i++)
    {
        balance_controller.current_sliderLocation[i].y = _s[i] / 360.f * 2.f * PI * 0.05;
        balance_controller.current_sliderSpeed[i].y = _sspeed[i] / 60.f * 2.f * PI * 0.05;
    }
}

/**
 * @brief  控制函数
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Chassis_Ctrl_Cal()
{
    //    static MeanFilter<50> speed_MF; //均值滤波
    static MeanFilter<20> turn_mf;
    static LowPassFilter turn_lf(0.5);
    float slider_s[2];
    float slider_sspeed[2];
    slider_s[LEFT] = Slider_Ctrl.slider[LEFT].getCurrentPos();
    slider_s[RIGHT] = Slider_Ctrl.slider[RIGHT].getCurrentPos();
    slider_sspeed[LEFT] = Slider_Ctrl.slider[LEFT].getCurrentSpeed();
    slider_sspeed[RIGHT] = Slider_Ctrl.slider[RIGHT].getCurrentSpeed();

    /*更新目标值*/
    Update_Target(gimbal_data.get_speed_y / 1000.0f, -gimbal_data.get_speed_z / 1000.0f, gimbal_data.get_speed_x / 1000.f);
    current_speed = (absWheelMotor[RIGHT].getMotorSpeed() + absWheelMotor[LEFT].getMotorSpeed()) / 2.0f * GEAR_SCALE;
    /*更新当前位姿*/
    Update_Current_Pos(absLpms.getEularData()->yaw, absLpms.getEularData()->pitch, gimbal_data.chassis_rotation_angle);
    /*更新当前速度*/
    Update_Current_Speed(current_speed, absLpms.getAngleVelData()->yaw, absLpms.getAngleVelData()->pitch);
    /*更新当前线性加速度*/
    Update_Current_Acc(absLpms.getAccData()->x, absLpms.getAccData()->y, absLpms.getAccData()->z);
    /*更新当前滑块状态*/
    Update_Slider_Params(slider_s, slider_sspeed);
    /*更新当前电机电流值*/
    Update_Motor_Current(Source_Current_Out);
    /*底盘控制*/
    balance_controller.Controller_Adjust();

    /* 输出合并，其中直立环和速度环互为反号 */
    wheel_stand_out_theory[LEFT] = balance_controller.Get_Data().stand_out + balance_controller.Get_Data().feedforward_out + balance_controller.Get_Data().distance_out + balance_controller.Get_Data().slider_out; //作为最后输出的一部分，直接转换成整数
    wheel_stand_out_theory[RIGHT] = balance_controller.Get_Data().stand_out + balance_controller.Get_Data().feedforward_out + balance_controller.Get_Data().distance_out + balance_controller.Get_Data().slider_out;

    wheel_speed_out_theory[LEFT] = balance_controller.Get_Data().speed_out - balance_controller.Get_Data().turn_out; //还需要做功率控制，先不转换类型
    wheel_speed_out_theory[RIGHT] = balance_controller.Get_Data().speed_out + balance_controller.Get_Data().turn_out;

    wheel_out_raw[LEFT] = (int)fabsf(wheel_speed_out_theory[LEFT]);
    wheel_out_raw[RIGHT] = (int)fabsf(wheel_speed_out_theory[RIGHT]);

    /*输出debug*/
    debug_A = balance_controller.Get_Data().stand_out;
    debug_B = balance_controller.Get_Data().feedforward_out;
    debug_C = balance_controller.Get_Data().distance_out;
    debug_D = balance_controller.Get_Data().speed_out;
    debug_E = balance_controller.Get_Data().turn_out;
    debug_F = slider_s[0];
}

/**
 * @brief 功率控制
 * @parma None
 * @return None
 */
// float motor_spd_scale;
void Balance_Infantry_Classdef::Power_Ctrl_Adjust()
{
    float power_get = Referee.GameRobotState.classis_power_limit;
    float power_target = chassis_power_limit;
    float power_current = digital_Power.power.pow_motor;
    static MeanFilter<50> target_mf;
    static MeanFilter<50> current_mf;
    current_mf << power_current;
    current_mf >> power_current;
    float energy_bias = power_target - power_current;
    power_energy += energy_bias * CTRL_INTERAL;
    // power_energy = std_lib::constrain(power_energy,POWER_ENERGY_MAX,0.f);
    if (power_energy > POWER_ENERGY_MAX)
    {
        power_energy = POWER_ENERGY_MAX;
    }
    /*设置目标速度挡位*/
    Set_MaxSpeed(power_get);
    /*设置充电功率*/
    Set_MaxPower(power_get);
    Power_Ctrl.Set_PE_Target(power_get, chassis_power_limit, 50);
    /* 仅对速度环和转向环进行功率控制 */
    Power_Ctrl.Control(digital_Power.power.pow_In, digital_Power.power.pow_motor, Referee.PowerHeatData.chassis_power_buffer, wheel_out_raw);
    power_limit_scale = std_lib::constrain((float)Power_Ctrl.Get_limScale(), 0.0f, 1.0f);
    debug_G = power_get;
}

/**
 * @brief 电源管理初始化
 * @parma None
 * @return None
 */
void Balance_Infantry_Classdef::Source_Init()
{
#if DIGITAL_POWER
    digital_Power.digital_Power_Init();
#else
    Source_Manage.SourceManage_Init();
#endif
}

/**
 * @brief 电源管理
 * @parma None
 * @return None
 */
void Balance_Infantry_Classdef::Source_Adjust()
{
#if DIGITAL_POWER
    //    vTaskDelay(1);
    //    Source_Manage.Update(Power_Ctrl.Get_capChargePower(), Referee.GameRobotState.classis_power_limit, Referee.PowerHeatData.chassis_power_buffer);
    //    Source_Manage.digital_Power_Control(Referee.GameRobotState.remain_HP);

    vTaskDelay(1); //不要改任务周期，也不要用vTaskDelayUntil
    // 传入1、底盘功率限制，2、当前缓冲能量
    digital_Power.Update(Referee.GameRobotState.classis_power_limit, Referee.PowerHeatData.chassis_power_buffer);
    // 传入当前血量，防止死亡后电容供电
    digital_Power.digital_Power_Control(Referee.GameRobotState.remain_HP);
    Source_Current_Out = digital_Power.DPW.I_out_H / 1000.f;
    Source_Cap_Voltage = digital_Power.DPW.Vcap / 1000.f;
#else
    vTaskDelay(10);
    Source_Manage.Update(ADCReadBuff);
    Source_Manage.Set_ChargePower(Power_Ctrl.Get_capChargePower());
    Source_Manage.Manage();
    HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, Source_Manage.capObj.charge_DAC_Value);
    Source_Current_Out = Source_Manage.current.cur_Out;
    Source_Cap_Voltage = Source_Manage.capObj.Voltage;
#endif
}

/**
 * @brief  控制量下发
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Chassis_Adjust()
{
    float wheel_out[2];
    /* 需要修改  高速时转向环响应不够 */
    if (fabsf(LPMS.get_data().Euler_Pitch) < 45.0f)
    {
        /* 固连自救，取消直立环作用，只利用速度环	自救 */
        if (gimbal_data.self_rescue_state)
        {
            wheel_stand_out_theory[LEFT] = 0;
            wheel_stand_out_theory[RIGHT] = 0;
            wheel_out[LEFT] = gimbal_data.get_speed_y / 1000.f * 2.f - balance_controller.Get_Data().turn_out;
            wheel_out[RIGHT] = gimbal_data.get_speed_y / 1000.f * 2.f + balance_controller.Get_Data().turn_out;
        }
        else
        {
            //制动取消速度环功率控制
            if (balance_controller.break_flag && !gimbal_data.rotation_state)
            {
                wheel_out[LEFT] = wheel_stand_out_theory[LEFT] + balance_controller.Get_Data().speed_out - balance_controller.Get_Data().turn_out * power_limit_scale;
                wheel_out[RIGHT] = wheel_stand_out_theory[RIGHT] + balance_controller.Get_Data().speed_out + balance_controller.Get_Data().turn_out * power_limit_scale;
            }
            else
            {
                if (power_energy < 0.f)
                {
                    wheel_out[LEFT] = wheel_stand_out_theory[LEFT] + balance_controller.Get_Data().speed_out * power_limit_scale - balance_controller.Get_Data().turn_out * power_limit_scale;
                    wheel_out[RIGHT] = wheel_stand_out_theory[RIGHT] + balance_controller.Get_Data().speed_out * power_limit_scale + balance_controller.Get_Data().turn_out * power_limit_scale;
                }
                else
                {
                    wheel_out[LEFT] = wheel_stand_out_theory[LEFT] + balance_controller.Get_Data().speed_out - balance_controller.Get_Data().turn_out * power_limit_scale;
                    wheel_out[RIGHT] = wheel_stand_out_theory[RIGHT] + balance_controller.Get_Data().speed_out + balance_controller.Get_Data().turn_out * power_limit_scale;
                }
            }
        }
        //防止电流过大
        if (Source_Current_Out > 10.0f)
        {
            wheel_out[LEFT] = std_lib::constrain(wheel_out[LEFT], -5.12f, 5.12f);
            wheel_out[RIGHT] = std_lib::constrain(wheel_out[RIGHT], -5.12f, 5.12f);
        }
        else
        {
            wheel_out[LEFT] = std_lib::constrain(wheel_out[LEFT], -5.12f, 5.12f);
            wheel_out[RIGHT] = std_lib::constrain(wheel_out[RIGHT], -5.12f, 5.12f);
        }
    }
    else
    {
        wheel_out[LEFT] = 0;
        wheel_out[RIGHT] = 0;
    }

    debug_F = wheel_out[LEFT];
    debug_G = wheel_out[RIGHT];

    if (balance_controller.weightless_flag == true)
    {
    }
    else
    {
        if (balance_controller.idling_flag == true)
        {
            wheel_out[LEFT] = 0;
            wheel_out[RIGHT] = 0;
        }
    }

    float rotation_slider_pos[2] = {-10, -10};
		if(gimbal_data.rotation_state)
		{
			rotation_slider_pos[0] = -10;
			rotation_slider_pos[1] = -10;
		}
		else if(gimbal_data.turn90degrees)
		{
			rotation_slider_pos[0] = -40;
			rotation_slider_pos[1] = -40;
		}
    if (gimbal_data.remote_ctrl_state == false)
    {
        wheel_out[LEFT] = 0;
        wheel_out[RIGHT] = 0;
        Slider_Ctrl.clear();
    }
    else
    {
        if (!gimbal_data.enable_cmd)
        {
            balance_controller.output.sliderCtrl_out[RIGHT] = 0;
            balance_controller.output.sliderCtrl_out[LEFT] = 0;
        }
        if (gimbal_data.rotation_state/* || gimbal_data.turn90degrees*/)
        {
            Slider_Ctrl.update(rotation_slider_pos);
            Slider_Ctrl.adjust();
            Slider_Ctrl.acutate();
        }
        else
        {
            // Slider_Ctrl.setTorqueOut(balance_controller.output.sliderCtrl_out);
            Slider_Ctrl.setVoltageOut(balance_controller.output.sliderCtrl_out);
        }
    }

    if (motorLinkCount[RIGHT] < 20 && motorLinkCount[LEFT] < 20)
    {
        absWheelMotor[LEFT].setMotorCurrentOut(std_lib::constrain(wheel_out[LEFT] * COM_9025_TORQUE_RATIO, -2000.f, 2000.f));
        absWheelMotor[RIGHT].setMotorCurrentOut(std_lib::constrain(wheel_out[RIGHT] * COM_9025_TORQUE_RATIO, -2000.f, 2000.f));
    }
    else
    {
        for (int i = 0; i < 2; i++)
        {
            if (motorLinkCount[i] >= 20)
                wheel_motor[i].startMotor();
            else
                wheel_motor[i].iqCloseControl_Current(0);
        }
    }

    /* 发送CAN包到云台 */
    Board_Com.gimbal_rx_pack2.chassis_flags &= 0xFFFE; //发送标志位除了第一位全部置1
    if ((current_state != &prebalance_state) && (current_state != &lostctrl_state))
    {
        Board_Com.gimbal_rx_pack2.chassis_flags |= 0x0001; //第一位置1（底盘正常）
    }
    if (Referee.GameState.stage_remain_time < 240 && Referee.GameState.stage_remain_time != 0)
        Board_Com.gimbal_rx_pack2.chassis_flags |= 0x0001 << 3; //第四位置1
    else
        Board_Com.gimbal_rx_pack2.chassis_flags &= ~(0x1 << 3); //第四位置0
    Board_Com.Send_GimbalPack1(board_queue, &Referee);
    Board_Com.Send_GimbalPack2(board_queue, &Referee, Source_Cap_Voltage * 7.f);
}

/**
 * @brief 切换速度挡位
 * @parma None
 * @return None
 */

void Balance_Infantry_Classdef::Set_MaxSpeed(uint16_t _powerMax)
/* 需测试 */
{
    if (_powerMax <= 60)
    {
        speed_scale = 1.8f;
    }
    else if (_powerMax > 60 && _powerMax < 100)
    {
        speed_scale = 2.1f;
    }
    else if (_powerMax >= 100)
    {
        speed_scale = 2.4f; // 0.5f
    }
    else
    {
        speed_scale = 2.1f;
    }
    /*各种功率标志位*/
    if (gimbal_data.leap_state && Source_Cap_Voltage > 17.0f)
    {
        speed_scale = 4.f;
    }
    else if (gimbal_data.unlimited_state && Source_Cap_Voltage > 17.0f)
    {
        speed_scale += 0.4f;
    }
    else if (gimbal_data.ascent_state && Source_Cap_Voltage > 17.0f)
    {
        speed_scale = 4.f;
    }
    else
    {
    }
    speed_scale = std_lib::constrain(speed_scale, -5.f, 5.f);
    if (Source_Cap_Voltage <= 14.f)
    {
        rotation_scale = 0.1;
    }
    else if (Source_Cap_Voltage <= 19.f && Source_Cap_Voltage > 14.f)
    {
        rotation_scale = 0.4;
    }
    else if (Source_Cap_Voltage > 19.f && Source_Cap_Voltage <= 23)
    {
        rotation_scale = 0.7f;
    }
    else
    {
        rotation_scale = 1.f;
    }
}

/**
 * @brief 设置充电功率
 * @parma None
 * @return None
 */
void Balance_Infantry_Classdef::Set_MaxPower(uint16_t _powerMax)
{
    /*滞回比较器，判断是否需要降功率充电*/
    static uint8_t compare_flag;
    if (Source_Cap_Voltage <= 17)
    {
        compare_flag = 0;
    }
    if (Source_Cap_Voltage >= 20)
    {
        compare_flag = 1;
    }
    if (compare_flag == 0)
    {
        chassis_power_limit = _powerMax - 30;
    }
    else if (compare_flag == 1)
    {
        chassis_power_limit = _powerMax;
    }
    /*超功率*/
    if (gimbal_data.unlimited_state && compare_flag)
    {
        chassis_power_limit += 20;
    }
    /*根据裁判系统切换功率*/
    switch (_powerMax)
    {
    case 60:
        chassis_power_limit -= 0;
        break;
    case 80:
        chassis_power_limit -= 0;
        break;
    case 100:
        chassis_power_limit -= 0;
        break;
    default:
        chassis_power_limit = 50;
        break;
    }
    if (gimbal_data.leap_state && compare_flag)
    {
        chassis_power_limit = 180;
    }
    if (gimbal_data.rotation_state)
    {
        chassis_power_limit = 55;
        if (compare_flag == 0)
        {
            chassis_power_limit = 45;
        }
    }
}

/**
 * @brief  连接检测函数
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Link_Check()
{
    for (int i = 0; i < 2; i++)
    {
        if (motorLinkCount[i] < 100)
            motorLinkCount[i]++;
    }
}

/**
 * @brief 9025电机状态获取
 * @parma None
 * @return None
 */
void Balance_Infantry_Classdef::Motor_State_Check()
{
    static float last_current[2] = {0, 0};
    wheel_motor[RIGHT].readMotorState1_errorState();
    wheel_motor[LEFT].readMotorState1_errorState();
    vTaskDelay(2);
    for (int i = 0; i < 2; i++)
    {
        float motor_current = wheel_motor[i].getData().current;
        if (wheel_motor[i].getData().errorState)
        {
            motorErrorCnt[i]++;
            wheel_motor[i].cleanErrorState();
            vTaskDelay(2);
            wheel_motor[i].startMotor();
            vTaskDelay(2);
            continue;
        }
        else
            motorErrorCnt[i] = 0;

        if (motor_current == last_current[i])
            motorDeadCnt[i]++;
        else
            motorDeadCnt[i] = 0;

        if (motorDeadCnt[i] >= 10)
        {
            wheel_motor[i].startMotor();
            vTaskDelay(2);
        }
        last_current[i] = motor_current;
    }
}

/**
 * @brief  底盘重置响应
 * @note
 * @param
 * @return
 * @retval  None
 */
void Balance_Infantry_Classdef::Reset_Adjust()
{
    //输出清零
    wheel_stand_out_theory[LEFT] = 0;
    wheel_stand_out_theory[RIGHT] = 0;
    wheel_speed_out_theory[LEFT] = 0;
    wheel_speed_out_theory[RIGHT] = 0;
    //目标值清零
    Update_Target(0, 0, 0);
    Update_Current_Pos(0, 0, 0);
    Update_Current_Speed(0, 0, 0);
    //控制器输出重置
    balance_controller.reset_adjust();
    //滑块重置
    Slider_Ctrl.clear();
}
