#ifndef _PUBLIC_DEFINE_H_
#define _PUBLIC_DEFINE_H_

#define WHEEL_NUM 2

#define RIGHT_WHEEL_ID 2
#define LEFT_WHEEL_ID 3

#define RIGHT_SLIDER_ID 4
#define LEFT_SLIDER_ID 1

#include "arm_math.h"

const float WHEEL_RADIUS = 0.105f;

const float ratio_RPM_to_RPS = 1 / 60.f;
const float ratio_torque_to_current = 1 / 4.6f * 13 / 20 * 16384;
const float ratio_degree_to_rad = 1 / 180.f * PI;

const float feedback_ratio = 13.7f * 9.8f * 0.12f / 2;
const float setpoint_rotation = -0.0f * ratio_degree_to_rad;
const float zero_point = -1.5f * ratio_degree_to_rad;
const float task_run_interval = 0.002f;

enum _chassis_WheelType {
	LEFT_JOINT = 1U,
	RIGHT_JOINT = 0U
};

//enum ControllerType
//{
//    FuzzyPID,
//    LQR
//};

/**
* @brief Chassis' Global Posture
*/
typedef struct _chassis_GlobalPos
{
    float x;
    float y;
    float roll;
    float pitch;
    float yaw;

    _chassis_GlobalPos& operator = (const _chassis_GlobalPos& value)
    {
        if (&value != this)
        {
            x = value.x; y = value.y; roll = value.roll; pitch = value.pitch; yaw = value.yaw;
        }
        return *this;
    }

    _chassis_GlobalPos& operator - (_chassis_GlobalPos& value) {
        x -= value.x;
        y -= value.y;
        roll -= value.roll;
        pitch -= value.pitch;
        yaw -= value.yaw;
        return *this;
    }
}chassis_GlobalPos;

/**
* @brief Chassis' Velocity
*/
typedef struct _chassis_Velocity
{
    float x_speed;
    float y_speed;
    float roll_angular_spd;
    float pitch_angular_spd;
    float yaw_angular_spd;

    _chassis_Velocity& operator = (_chassis_Velocity& value) {
        x_speed = value.x_speed;
        y_speed = value.y_speed;
        roll_angular_spd = value.roll_angular_spd;
        pitch_angular_spd = value.pitch_angular_spd;
        yaw_angular_spd = value.yaw_angular_spd;
        return *this;
    }

    _chassis_Velocity& operator + (_chassis_Velocity& value) {
        x_speed += value.x_speed;
        y_speed += value.y_speed;
        roll_angular_spd += value.roll_angular_spd;
        pitch_angular_spd += value.pitch_angular_spd;
        yaw_angular_spd += value.yaw_angular_spd;
        return *this;
    }

    _chassis_Velocity& operator * (float _scale) {
        x_speed *= _scale;
        y_speed *= _scale;
        roll_angular_spd *= _scale;
        pitch_angular_spd *= _scale;
        yaw_angular_spd *= _scale;
        return *this;
    }
}chassis_Velocity;

#endif // !_PUBLIC_DEFINE_H_
