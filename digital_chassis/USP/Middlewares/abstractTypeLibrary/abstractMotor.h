/**
 ******************************************************************************
 * Copyright (c) 2023 - ~, SCUT-RobotLab Development Team
 * @file abstractMotor.h
 * @author 余俊晖 (2460857175@qq.com)
 * @brief 抽象电机库，用于对接顶层计算得控制量与底层电机的实际输出
 *        方便将仿真用的理想顶层，对接到实车
 * @version 1.0
 * @date 2023-03-04
 *
 ******************************************************************************
 * @attention
 *
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version Number, write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 *
 * <h2><center>&copy; Copyright (c) 2023 - ~, SCUT-RobotLab Development Team.
 * All rights reserved.</center></h2>
 ******************************************************************************
 */
#ifndef ABSTRACTMOTOR_H
#define ABSTRACTMOTOR_H

#define USE_DJI_MOTOR 1
#define USE_MF9025 1
#define USE_HT04 0

#if USE_DJI_MOTOR
#include "SRML.h"
#endif

#if USE_HT04
#include "HT04.h"
#endif

#if USE_MF9025
#include <MF9025_v2.h>
#endif

namespace abMotor
{
    /**
     * @brief 电机抽象基类，用于对接顶层计算得控制量与底层电机的实际输出
     * 在继承类中，加入电机类的指针成员，绑定电机变量，定义几个纯虚函数，即可正常使用
     */
    class abstractMotorBase
    {
    protected:
        virtual float getRawMotorTotalAngle() = 0;
        virtual float getRawMotorAngle() = 0;
        virtual float getRawMotorSpeed() = 0;
        virtual void setRawMotorCurrentOut(float out) = 0;

    public:
        virtual float getMotorTotalAngle()
        {
            return getRawMotorTotalAngle() * angle_unit_convert * Polarity + baseAngle;
        }

        virtual float getMotorAngle()
        {
            return getRawMotorAngle() * angle_unit_convert * Polarity + baseAngle;
        }

        virtual float getMotorSpeed()
        {
            return getRawMotorSpeed() * speed_unit_convert * Polarity;
        }

        virtual void setMotorCurrentOut(float out)
        {
            setRawMotorCurrentOut(out * out_unit_convert * Polarity);
        }

        // 极性，将电机实际转动方向的正负，对齐到人为设置的方向，自行设置
        // 对于绝大部分电机来说，输出正力矩时，速度增加，角度增加，所以三者极性是具有统一性的
        // 因此，只需要设置一个极性即可
        float Polarity = 1;

        float speed_unit_convert = 1; // 获取数据的单位转换，自行设置

        float angle_unit_convert = 1; // 获取数据的单位转换，自行设置
        float baseAngle = 0;          // 角度基础值，会在极性对齐、单位转换后，再加上这个值

        // 如果需要将顶层下发的力矩，转换为电流，再用电机类下发，就需要设置这个值
        // 如果不需要单位转换，就不用管
        float out_unit_convert = 1;
    };
}

/**
 * @brief 电机抽象模板类，仅仅为了可以特化而写，并无实际作用
 * 用模板是为了不用给所有类型的电机抽象类都设置一个类名
 * 模板主类不写实现，是为了避免在传入没有特化过的电机类型时，会出现无法设想的错误
 * @tparam motorType
 */
template <class motorType>
class abstractMotor
{
};

#if USE_HT04
/**
 * @brief HT04电机抽象类
 */
template <>
class abstractMotor<MotorHT04Classdef> : public abMotor::abstractMotorBase
{
public:
    CAN_COB Tx_Buff;
    QueueHandle_t CAN_TxPort = nullptr;
    MotorHT04Classdef *motor = nullptr;
    virtual float getRawMotorTotalAngle() { return 0; }
    virtual float getRawMotorAngle()
    {
        if (motor == nullptr)
            return 0;
        return motor->getRecData().position;
    }
    virtual float getRawMotorSpeed()
    {
        if (motor == nullptr)
            return 0;
        return motor->getRecData().velocity;
    }
    virtual void setRawMotorCurrentOut(float out)
    {
        if (motor == nullptr)
            return;
        motor->setTorque(out);
        MotorMsgPack(Tx_Buff, *motor);
        xQueueSend(CAN_TxPort, &Tx_Buff, 0);
    }
    virtual void setRawMotorAngle(float angle, float kp, float kd)
    {
        if (motor == nullptr)
            return;
        motor->setPosition(angle, kp, kd);
        MotorMsgPack(Tx_Buff, *motor);
        xQueueSend(CAN_TxPort, &Tx_Buff, 0);
    }
    virtual void setRawMotorSpeed(float speed, float kd, float torque)
    {
        if (motor == nullptr)
            return;
        motor->setVelocity(speed, kd, torque);
        MotorMsgPack(Tx_Buff, *motor);
        xQueueSend(CAN_TxPort, &Tx_Buff, 0);
    }

public:
    void init(QueueHandle_t _CAN_TxPort, MotorHT04Classdef *_motor)
    {
        CAN_TxPort = _CAN_TxPort;
        motor = _motor;

        motor->cmd_motor_mode(Tx_Buff);
        xQueueSend(CAN_TxPort, &Tx_Buff, 0);
    }
    virtual void setMotorSpeed(float speed, float kd, float torque)
    {
        setRawMotorSpeed(speed / speed_unit_convert / Polarity, kd, torque / out_unit_convert / Polarity);
    }
    virtual void setMotorAngle(float angle, float kp, float kd)
    {
        setRawMotorAngle((angle - baseAngle) / angle_unit_convert / Polarity, kp, kd);
    }
};
#endif

#if USE_MF9025
/**
 * @brief MF9025_v2电机抽象类
 */
template <>
class abstractMotor<MotorMF9025v2Classdef> : public abMotor::abstractMotorBase
{
private:
    MotorMF9025v2Classdef *motor = nullptr;
    virtual float getRawMotorTotalAngle()
    {
        if (motor == nullptr)
            return 0;
        return motor->getData().totalAngleLocal;
    }
    virtual float getRawMotorAngle()
    {
        if (motor == nullptr)
            return 0;
        return motor->getData().singleAngle;
    }
    virtual float getRawMotorSpeed()
    {
        if (motor == nullptr)
            return 0;
        return motor->getData().speed;
    }
    virtual void setRawMotorCurrentOut(float out)
    {
        if (motor == nullptr)
            return;
        motor->iqCloseControl_Current(out);
    }
    virtual void setRawMotorAngle(float angle, float kp, float kd) {}
    virtual void setRawMotorSpeed(float speed, float kd, float torque) {}

public:
    void bindMotor(MotorMF9025v2Classdef *_motor) { motor = _motor; }
};
#endif

#if USE_DJI_MOTOR
template <>
class abstractMotor<Motor_C620> : public abMotor::abstractMotorBase
{
public:
    Motor_C620 *motor = nullptr;
    void bindMotor(Motor_C620 *_motor) { motor = _motor; }
    bool CheckID(uint32_t id)
    {
        if (motor == nullptr)
            return 0;
        return motor->CheckID(id);
    }
    void update(uint8_t can_rx_data[])
    {
        if (motor == nullptr)
            return;
        motor->update(can_rx_data);
    }

private:
    virtual float getRawMotorTotalAngle()
    {
        if (motor == nullptr)
            return 0;
        return motor->getAngle();
    };
    virtual float getRawMotorAngle()
    {
        if (motor == nullptr)
            return 0;
        return motor->getEncoder() / 8192.f * 360;
    };
    virtual float getRawMotorSpeed()
    {
        if (motor == nullptr)
            return 0;
        return motor->getSpeed();
    };
    virtual void setRawMotorCurrentOut(float out)
    {
        if (motor == nullptr)
            return;
        motor->Out = out;
    };
};

template <>
class abstractMotor<Motor_C610> : public abMotor::abstractMotorBase
{
public:
    Motor_C610 *motor = nullptr;
    void bindMotor(Motor_C610 *_motor) { motor = _motor; }
    bool CheckID(uint32_t id)
    {
        if (motor == nullptr)
            return 0;
        return motor->CheckID(id);
    }
    void update(uint8_t can_rx_data[])
    {
        if (motor == nullptr)
            return;
        motor->update(can_rx_data);
    }

private:
    virtual float getRawMotorTotalAngle()
    {
        if (motor == nullptr)
            return 0;
        return motor->getAngle();
    };
    virtual float getRawMotorAngle()
    {
        if (motor == nullptr)
            return 0;
        return motor->getEncoder() / 8192.f * 360;
    };
    virtual float getRawMotorSpeed()
    {
        if (motor == nullptr)
            return 0;
        return motor->getSpeed();
    };
    virtual void setRawMotorCurrentOut(float out)
    {
        if (motor == nullptr)
            return;
        motor->Out = out;
    };
};
#endif

#endif
