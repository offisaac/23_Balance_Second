#ifndef _SLIDING_BLOCK_H_
#define _SLIDING_BLOCK_H_

#include "abstractMotor.h"
#include "public_define.h"

namespace SliderNameSpace
{
    template <class motorType>
    class SliderBaseClassdef
    {
    protected:
        float posRatio = 1;
        float targetPos = 0, currentPos = 0;
        float ratioGear = 1.f; // 减速比
        float posMax = 100.f;  // 滑块最大量程
        abstractMotor<motorType> absMotor;

    public:
        motorType motor;
        myPID positionLoop, speedLoop;

        SliderBaseClassdef(uint8_t id, int8_t _Polarity, float _posRatio = 1.f, float _ratioGear = 1.f, float _posMax = 100.f)
            : motor(id)
        {
            absMotor.bindMotor(&motor);
            setPolarity(_Polarity);
            setPosRatio(_posRatio);
            ratioGear = fabsf(_ratioGear);
            posMax = fabsf(_posMax);

            absMotor.speed_unit_convert = 1 / ratioGear;
        }
        virtual void setPosRatio(float _posRatio)
        {
            posRatio = _posRatio;
            absMotor.angle_unit_convert = 1 / ratioGear * posRatio;
        }

        virtual void setOffset(float _offset) { absMotor.baseAngle = -_offset; }
        virtual float getCurrentSpeed() { return absMotor.getMotorSpeed(); }
        virtual float getCurrentPos() { return currentPos = absMotor.getMotorTotalAngle(); }

        virtual void setPolarity(int8_t _Polarity)
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

        virtual void controlSpeed(float targetVel)
        {
            speedLoop.Target = targetVel;
            speedLoop.Current = absMotor.getMotorSpeed();
            speedLoop.Adjust();
            absMotor.setMotorCurrentOut(speedLoop.Out);
        }

        virtual void update(float _targetPos)
        {
            targetPos = std_lib::constrain(_targetPos, -posMax, posMax);
            currentPos = absMotor.getMotorTotalAngle();
        }

        virtual void adjust()
        {
            positionLoop.Target = targetPos;
            positionLoop.Current = currentPos;

            speedLoop.Target = positionLoop.Adjust();
            speedLoop.Current = absMotor.getMotorSpeed();
            speedLoop.Adjust();

            absMotor.setMotorCurrentOut(speedLoop.Out);
        }

        virtual void setCurrentOut(float _out)
        {
            absMotor.setMotorCurrentOut(_out);
        }

        virtual void clearCommand()
        {
            absMotor.setMotorCurrentOut(0);
        }
    };
}

template <class motorType>
class SliderClassdef : public SliderNameSpace::SliderBaseClassdef<motorType>
{
public:
    SliderClassdef(uint8_t id, int8_t _Polarity, float _posRatio = 1.f, float _ratioGear = 1.f, float _posMax = 100.f)
        : SliderNameSpace::SliderBaseClassdef<motorType>(id, _Polarity, _posRatio, _ratioGear, _posMax)
    {
    }
};

template <>
class SliderClassdef<Motor_GM6020> : public SliderNameSpace::SliderBaseClassdef<Motor_GM6020>
{
public:
    SliderClassdef(uint8_t id, int8_t _Polarity, float _posRatio = 1.f, float _ratioGear = 1.f, float _posMax = 100.f)
        : SliderNameSpace::SliderBaseClassdef<Motor_GM6020>(id, _Polarity, _posRatio, _ratioGear, _posMax)
    {
    }

    void setEncoderOffset(uint16_t _offset) { absMotor.setEncoderOffset(_offset); }

    void setVoltageOut(float _out)
    {
        absMotor.setMotorVoltageOut(_out);
    }

    void setTorqueOut(float torque)
    {
        absMotor.setMotorTorqueOut(torque);
    }

    virtual void clearCommand() override
    {
        // absMotor.setMotorTorqueOut(0);
        absMotor.setMotorVoltageOut(0);
        absMotor.clearCurrentLoop();
    }
};
#endif
