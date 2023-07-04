#ifndef _BALANCE_CHASSIS_H_
#define _BALANCE_CHASSIS_H_

#define DIGITAL_POWER 1

#include "SRML.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "math.h"
#include "board_com.h"
#include "LPMS_BE2.h"
#include "balance_controller.h"
#include "Slider_controller.h"
#include "MF9025_v2.h"
#include "abstractMotor.h"
#include "abstractIMU.h"

#if DIGITAL_POWER
#include "digital_Power.h"
#else
#include "source_manage.h"
#endif

#define WHEEL_R 0.110f                               //轮子半径（m）
#define GEAR_SCALE 2.0f * PI * WHEEL_R / 1.0f / 60.0f //转速(rpm)转换为线速度(m/s)参数
#define RPS_TO_RPM 1.0f * 60.0f / (2 * PI)            //把陀螺仪角速度（rps）转化为转速(rpm)
#define CTRL_INTERAL 0.003f                          //控制周期
#define POWER_ENERGY_MAX 150.0f
#define DEGREE_TO_RAD PI / 180.f
#define COM_9025_TORQUE_RATIO 1.f / 0.32f * 2048.f / 16.5f

//左右电机编号
enum _chassis_WheelEnumdef
{
    LEFT = 1U,
    RIGHT = 0U
};

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

//云台数据结构体
#pragma pack(1)
typedef struct _Gimbal_Data_Structdef
{
    int16_t get_speed_y;
    int16_t get_speed_x;
    int16_t get_speed_z;
    /* 更新标志位 */
    uint8_t remote_ctrl_state : 1; //遥控状态
    uint8_t unlimited_state : 1;   //超功率
    uint8_t rotation_state : 1;    //小陀螺
    uint8_t leap_state : 1;        //飞坡
    uint8_t bulletbay_state : 1;   //弹舱盖开启
    uint8_t ascent_state : 1;      //上坡
    uint8_t ui_reset_flag : 1;     //手动复位
    uint8_t vision_mode_flag1 : 1; //视觉模式1
    uint8_t vision_mode_flag2 : 1; //视觉模式2
    uint8_t enable_cmd : 1;        //底盘使能
    uint8_t self_rescue_state : 1; //固连自救
    uint8_t sliding_remake : 1;    //滑块复位
    uint8_t turn90degrees : 1;     //转90度
    uint8_t gg_flag : 1;           //寄掉
    uint8_t vision_can_shoot : 1;  //视觉是否发射
    uint8_t fri_state : 1;         //摩擦轮是否打开

    /*底盘旋转角*/
    float chassis_rotation_angle;
} Gimbal_Data_Structdef;
#pragma pack()

//平衡步大类
class Balance_Infantry_Classdef
{
public:
    Balance_Infantry_Classdef();
    //总控函数
    void Chassis_Ctrl()
    {
        Judge_State();       // 执行状态机
        Power_Ctrl_Adjust(); //功率控制
        Chassis_Adjust();    //控制量下发
    }
    //配置函数
    void Load_Chassis_Queue(QueueHandle_t *_motor_queue, QueueHandle_t *_board_queue);
    // void Load_Balance_Controller(float (*pFunc)(const float pos_current, const float pos_target,const float speed_current,const float speed_target));
    //通信函数
    void Gimbal_Data_Update(CAN_COB *CAN_RxMsg); //板间通信
    void CAN_Motor_Update(CAN_COB *CAN_RxMsg);   //电机通信
    void UART_Gyro_Update(uint8_t *_rx_msg);     //陀螺仪通信
    //电源控制
    void Source_Init();
    void Source_Adjust(); //电源管理控制
    //引入类型
    MotorMF9025v2Classdef wheel_motor[2] = {MotorMF9025v2Classdef(2), MotorMF9025v2Classdef(1)};
    abstractMotor<MotorMF9025v2Classdef> absWheelMotor[2];
    abstractIMUClassdef<LPMS_BE2_Typedef> absLpms;
    Controller<LQR> balance_controller; //底盘控制器
#if DIGITAL_POWER
    // digital_Power_Classdef Source_Manage; //数字电源
#else
    C_SourceManage_Classdef Source_Manage; //电源开关
#endif
    referee_Classdef Referee;                           //裁判系统管理
    GimbalCom_Classdef Board_Com;                       //裁判系统交互
    PowerCtrl_ClassDef Power_Ctrl;                      //功率控制
    LPMS_BE2_Typedef LPMS;                              //陀螺仪
    SliderControllerClassdef<Motor_GM6020> Slider_Ctrl; //滑块控制器

    //切换状态函数
    void Status_Switching(State_Base *_state)
    {
        this->current_state = _state;           //状态指针指向新的状态
        this->current_state->Set_Context(this); //将当前数据挂载到状态机当中
    }
    float power_energy = POWER_ENERGY_MAX; //引入功率能量控制
    float chassis_power_limit;             //底盘功率限制

public:
    //使用友元类，使状态方法可以配置大类数据
    friend Lost_Ctrl_State;
    friend Pre_Balance_State;
    friend Balance_State;

    State_Base *current_state; //状态基类指针，用于更新状态

    //总控函数部分
    void Judge_State();       //转换状态
    void Chassis_Ctrl_Cal();  //控制器计算
    void Power_Ctrl_Adjust(); //功率控制
    void Chassis_Adjust();    //控制量下发
    void Reset_Adjust();      //重置函数
    //控制器更新数据部分
    void Update_Target(float _y_speed, float _z_speed, float _x_speed);
    void Update_Current_Pos(float _yaw, float _pitch, float _chassis_angle);
    void Update_Current_Speed(float _y, float _yaw, float _pitch);
    void Update_Current_Acc(float _x, float _y, float _z);
    void Update_Motor_Current(float _current);
    //功率控制部分
    float Source_Current_Out = 0;
    float Source_Cap_Voltage = 0;

    float speed_scale = 0;    //速度系数，用于限制当前功率下的速度
    float rotation_scale = 0; //小陀螺转速，用于限制不同功率状况下的转速
    float max_wheel_speed;    //轮子最大转速
    float max_launch_speed;   //起步最大转速
    float max_wheel_output;   //轮子最大输出电流
    float power_limit_scale;  //功率控制输出的功率系数，最后对速度输出做限制
    float fact_target_speed;
    void Set_MaxSpeed(uint16_t _powerMax); //设置最大速度，其实就是配置speed_scale
    void Set_MaxPower(uint16_t _powerMax); //设置底盘最大功率，包含功率策略

    //通信及其变量部分
    Gimbal_Data_Structdef gimbal_data; //云台数据结构体
    uint8_t auto_mode;                 //视觉模式
    QueueHandle_t *board_queue;        //板件通信队列接口
    /*底盘控制器*/
    // float (*balance_controller)(const float pos_current, const float pos_target,const float speed_current,const float speed_target) = NULL;
    //底盘输出
    float wheel_stand_out_theory[2] = {0}; //直立环理论输出（相对于整车的前后而言）
    float wheel_speed_out_theory[2] = {0}; //速度环理论输出（相对于整车的前后而言）
    int wheel_out_raw[2] = {0};            //电机功控输出  （相对于单个轮子而言）
    float current_speed = 0;               //获取当前底盘速度

    /*小陀螺平移*/
    float angle_error = 0.f;          //目标速度向量与底盘坐标角度差
    float rotation_move_gain = 0.3f;  //平移速度增益
    float target_rotation_r = 0;      //目标速度模长
    float target_rotation_angle = 0;  //速度向量旋转角
    float rotation_chassis_angle = 0; //底盘坐标相对于云台转角
};

#endif
