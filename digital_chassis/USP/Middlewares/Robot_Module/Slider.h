#ifndef _SLIDING_BLOCK_H_
#define _SLIDING_BLOCK_H_

#include "abstractMotor.h"
#include "public_define.h"

class SliderClassdef
{
private:
    float posRatio = 1;
    float targetPos = 0, currentPos = 0;
    abstractMotor<Motor_C610> absMotor;

public:
    Motor_C610 motor;
    myPID positionLoop, speedLoop;

    SliderClassdef(uint8_t id, int8_t _Polarity = 1, float _posRatio = 1);
    void setPosRatio(float _posRatio)
    {
        posRatio = _posRatio;
        absMotor.angle_unit_convert = 1 / 36.f * posRatio;
    }
    void setPolarity(int8_t _Polarity);
    void setOffset(float _offset) { absMotor.baseAngle = -_offset; }
    float getCurrentSpeed() { return absMotor.getMotorSpeed(); }
    float getCurrentPos() { return currentPos = absMotor.getMotorTotalAngle(); }

    void controlSpeed(float targetVel); // 速度控制，用于初始化使用

    void update(float _targetPos);
    void adjust();
    void clearCommand(); //清除滑块控制量，关闭输出
};

#endif
