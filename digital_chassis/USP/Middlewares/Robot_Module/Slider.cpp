#include "Slider.h"

SliderClassdef::SliderClassdef(uint8_t id, int8_t _Polarity, float _posRatio)
    : motor(id)
{
    absMotor.bindMotor(&motor);
    setPolarity(_Polarity);
    setPosRatio(_posRatio);

    absMotor.speed_unit_convert = 1 / 36.f;
}

void SliderClassdef::setPolarity(int8_t _Polarity)
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

void SliderClassdef::controlSpeed(float targetVel)
{
    speedLoop.Target = targetVel;
    speedLoop.Current = absMotor.getMotorSpeed();
    speedLoop.Adjust();
    absMotor.setMotorCurrentOut(speedLoop.Out);
}

void SliderClassdef::update(float _targetPos)
{
    targetPos = _targetPos;
    currentPos = absMotor.getMotorTotalAngle();
}

void SliderClassdef::adjust()
{
    positionLoop.Target = targetPos;
    positionLoop.Current = currentPos;

    speedLoop.Target = positionLoop.Adjust();
    speedLoop.Current = absMotor.getMotorSpeed();
    speedLoop.Adjust();

    absMotor.setMotorCurrentOut(speedLoop.Out);
}

void SliderClassdef::clearCommand()
{
    // positionLoop.Target = 0;
    // positionLoop.Current = 0;
    // positionLoop.I_Term = 0;

    // speedLoop.Target = 0;
    // speedLoop.Current = 0;
    // speedLoop.I_Term = 0;

    absMotor.setMotorCurrentOut(0);
}
