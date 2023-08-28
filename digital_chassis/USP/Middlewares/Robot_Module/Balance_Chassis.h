#ifndef _BALANCE_CHASSIS_H_
#define _BALANCE_CHASSIS_H_

#define DIGITAL_POWER 1

#include "SRML.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "math.h"
#include "board_com.h"
#include "balance_controller.h"

#define WHEEL_R 0.110f                               //轮子半径（m）
#define GEAR_SCALE 2.0f * PI *WHEEL_R / 1.0f / 60.0f //转速(rpm)转换为线速度(m/s)参数
#define RPS_TO_RPM 1.0f * 60.0f / (2 * PI)           //把陀螺仪角速度（rps）转化为转速(rpm)
#define CTRL_INTERAL 0.003f                          //控制周期
#define POWER_ENERGY_MAX 150.0f
#define DEGREE_TO_RAD PI / 180.f
#define COM_9025_TORQUE_RATIO 1.f / 0.32f * 2048.f / 16.5f


//声明平衡步兵类
class Balance_Infantry_Classdef;

//状态基类
class State_Base
{
protected:
    Balance_Infantry_Classdef *context; //上下文指针，指向要操作的类对象

public:
    //上下文切换，让实例化对象获取被控对象的指针（索引）
    void Set_Context(Balance_Infantry_Classdef *_context)
    {
        this->context = _context;
    }

    virtual void State_Handler() = 0; //状态机方法，子类中实现实例化
};

//失控状态机
class Lost_Ctrl_State : public State_Base
{
    virtual void State_Handler();
};

//预平衡状态机
class Pre_Balance_State : public State_Base
{
    virtual void State_Handler();
};

//平衡状态机
class Balance_State : public State_Base
{
    virtual void State_Handler();
};

extern Lost_Ctrl_State lostctrl_state;
extern Pre_Balance_State prebalance_state;
extern Balance_State balance_state;

//平衡步大类
class Balance_Infantry_Classdef
{
public:
    Balance_Infantry_Classdef();
    //总控函数
    void Chassis_Ctrl()
    {
        Judge_State();       // 执行状态机
        Chassis_Adjust();    //控制量下发
    }

    //引入类型
    Controller<LQR> balance_controller; //底盘控制器

    //切换状态函数
    inline void Status_Switching(State_Base *_state)
    {
        this->current_state = _state;           //状态指针指向新的状态
        this->current_state->Set_Context(this); //将当前数据挂载到状态机当中
    }
		
	int16_t machine_mode = 0;

public:
    //使用友元类，使状态方法可以配置大类数据
    friend Lost_Ctrl_State;
    friend Pre_Balance_State;
    friend Balance_State;

    State_Base *current_state; //状态基类指针，用于更新状态

    //总控函数部分
    void Judge_State();       //转换状态
    void Chassis_Ctrl_Cal();  //控制器计算
    void Chassis_Adjust();    //控制量下发
    void Link_Check();        //通信检测函数
    void Motor_State_Check(); //电机状态检测与纠正
    void Reset_Adjust();      //重置函数
    //控制器更新数据部分
    void Update_Target(float _y_speed, float _z_speed, float _x_speed);
    void Update_Current_Pos(float _yaw, float _pitch, float _chassis_angle);
    void Update_Current_Speed(float _y, float _yaw, float _pitch);
    void Update_Current_Acc(float _x, float _y, float _z);
    void Update_Motor_Current(float _current);
    void Update_Slider_Params(float _s[2], float _sspeed[2]);
    //功率控制部分
    float Source_Cap_Voltage = 0;

    float speed_scale = 0;    //速度系数，用于限制当前功率下的速度
    float rotation_scale = 0; //小陀螺转速，用于限制不同功率状况下的转速
    float max_wheel_speed;    //轮子最大转速
    float max_launch_speed;   //起步最大转速
    float max_wheel_output;   //轮子最大输出电流
    float fact_target_speed;
    void Set_MaxSpeed(uint16_t _powerMax); //设置最大速度，其实就是配置speed_scale

    //底盘输出
    float wheel_stand_out_theory[2] = {0}; //直立环理论输出（相对于整车的前后而言）
    float wheel_speed_out_theory[2] = {0}; //速度环理论输出（相对于整车的前后而言）
    int wheel_out_raw[2] = {0};            //电机功控输出  （相对于单个轮子而言）
	float wheel_out[2];

    /*小陀螺平移*/
    float angle_error = 0.f;          //目标速度向量与底盘坐标角度差
    float rotation_move_gain = 0.3f;  //平移速度增益
    float target_rotation_r = 0;      //目标速度模长
    float target_rotation_angle = 0;  //速度向量旋转角
    float rotation_chassis_angle = 0; //底盘坐标相对于云台转角

    //电机状态检测
    uint8_t motorLinkCount[2] = {}; // 电机连接检测1
    uint16_t motorErrorCnt[2] = {}; // 电机出错统计
    uint16_t motorDeadCnt[2] = {};
};

#endif
