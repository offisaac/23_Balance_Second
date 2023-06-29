#ifndef _SLIDING_BLOCK_H_
#define _SLIDING_BLOCK_H_

#include "abstractMotor.h"
#include "public_define.h"

template <class motorType>
class SliderClassdef
{
private:
    float posRatio = 1;
    float targetPos = 0, currentPos = 0;
    float ratioGear = 1.f; //减速比
    abstractMotor<motorType> absMotor;

public:
    motorType motor;
    myPID positionLoop, speedLoop;

    SliderClassdef(uint8_t id, int8_t _Polarity, float _posRatio, float _ratioGear)
        : motor(id)
    {
        absMotor.bindMotor(&motor);
        setPolarity(_Polarity);
        setPosRatio(_posRatio);
        ratioGear = fabsf(_ratioGear);

        absMotor.speed_unit_convert = 1 / ratioGear;
    }
    void setPosRatio(float _posRatio)
    {
        posRatio = _posRatio;
        absMotor.angle_unit_convert = 1 / ratioGear * posRatio;
    }

    void setOffset(float _offset) { absMotor.baseAngle = -_offset; }
    void setEncoderOffset(uint16_t _offset) { absMotor.setEncoderOffset(_offset); }
    float getCurrentSpeed() { return absMotor.getMotorSpeed(); }
    float getCurrentPos() { return currentPos = absMotor.getMotorTotalAngle(); }

    void setPolarity(int8_t _Polarity)
    {
        if (_Polarity != 1 && _Polarity != -1)
        {
            return;
        }
        else
        {
            absMotor.Polarity = _Polarity;
            return;
        }
    }

    void controlSpeed(float targetVel)
    {
        speedLoop.Target = targetVel;
        speedLoop.Current = absMotor.getMotorSpeed();
        speedLoop.Adjust();
        absMotor.setMotorCurrentOut(speedLoop.Out);
    }

    void update(float _targetPos)
    {
        targetPos = _targetPos;
        currentPos = absMotor.getMotorTotalAngle();
    }

    void adjust()
    {
        positionLoop.Target = targetPos;
        positionLoop.Current = currentPos;

        speedLoop.Target = positionLoop.Adjust();
        speedLoop.Current = absMotor.getMotorSpeed();
        speedLoop.Adjust();

        absMotor.setMotorCurrentOut(speedLoop.Out);
    }

    void clearCommand()
    {
        // positionLoop.Target = 0;
        // positionLoop.Current = 0;
        // positionLoop.I_Term = 0;

        // speedLoop.Target = 0;
        // speedLoop.Current = 0;
        // speedLoop.I_Term = 0;

        absMotor.setMotorCurrentOut(0);
    }
};

#endif
