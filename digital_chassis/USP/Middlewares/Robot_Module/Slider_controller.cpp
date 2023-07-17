#include "Slider_controller.h"

//////////////////////////////////////////// 2006滑块 //////////////////////////////////////////////////////////
SliderControllerClassdef<Motor_C610>::SliderControllerClassdef()
{
    slider[RIGHT_JOINT].positionLoop.SetPIDParam(5, 0, 0, 10, 300);
    slider[LEFT_JOINT].positionLoop.SetPIDParam(5, 0, 0, 10, 300);

    slider[LEFT_JOINT].speedLoop.SetPIDParam(50, 1, 0, 2000, 10000);
    slider[RIGHT_JOINT].speedLoop.SetPIDParam(50, 1, 0, 2000, 10000);
}

void SliderControllerClassdef<Motor_C610>::init()
{
    float originOutMax[2];
    uint16_t cnt[2] = {200, 200};
    bool initFlag[2] = {false, false};
    int8_t initDirection[2] = {-1, -1};
    float offset[2][2] = {{0, 0}, {0, 0}};

    originOutMax[0] = slider[0].speedLoop.Out_Max;
    originOutMax[1] = slider[1].speedLoop.Out_Max;

    //防止同步带被拉太长而受到损伤
    slider[LEFT_JOINT].speedLoop.Out_Max = 5000;
    slider[RIGHT_JOINT].speedLoop.Out_Max = 5000;
    for (;;)
    {
        if (sliderMotorInit[0] == true && sliderMotorInit[1] == true)
        {
            for (int i = 0; i < 2; i++)
            {
                if (slider[i].getCurrentSpeed() < 0.3f) //堵转判断
                {
                    if (cnt[i] > 0) //计数器自减
                        cnt[i]--;
                    else
                        cnt[i] = 0;
                }
                else
                {
                    cnt[i] = 150;
                }

                if (cnt[i] != 0) //如果不堵转就以恒速向一个方向运动
                {
                    slider[i].controlSpeed(300 * initDirection[i]);
                }
                else
                {
                    if (initDirection[i] == 1)
                    {
                        offset[i][1] = slider[i].getCurrentPos() - 5;
                        if (initFlag[i] == false) //第一次设置初始化设置初始位置
                        {
                            slider[i].setOffset((offset[i][0] + offset[i][1]) / 2.f); // 因为初始化位置卡死时，同步带会被拉长
                            initFlag[i] = true;
                        }
                        else
                        {
                            slider[i].controlSpeed(0); //已经初始化后就关闭输出
                        }
                    }
                    else
                    {
                        offset[i][0] = slider[i].getCurrentPos() + 5;
                        initDirection[i] = 1;
                        cnt[i] = 150;
                    }
                }
            }

            if (initFlag[0] == true && initFlag[1] == true)
            {
                slider[0].speedLoop.Out_Max = originOutMax[0];
                slider[1].speedLoop.Out_Max = originOutMax[1];
                return;
            }

            acutate();
            vTaskDelay(2);
        }
        else
        {
        }
    }
}

void SliderControllerClassdef<Motor_C610>::update(float targetPos[2])
{
    for (int i = 0; i < 2; i++)
    {
        slider[i].update(targetPos[i]);
    }
}

void SliderControllerClassdef<Motor_C610>::adjust()
{
    for (int i = 0; i < 2; i++)
    {
        slider[i].adjust();
    }
}

void SliderControllerClassdef<Motor_C610>::acutate()
{
    if (canTxPort == nullptr)
    {
        return;
    }

    canTxPack = MotorMsgPack(canTxPack, slider[0].motor, slider[1].motor);
    xQueueSend(canTxPort, &(canTxPack.Id200), 0);
}

void SliderControllerClassdef<Motor_C610>::updateMotorData(CAN_COB *CAN_RxMsg)
{
    if (slider[0].motor.CheckID(CAN_RxMsg->ID))
    {
        slider[0].motor.update(CAN_RxMsg->Data);
        sliderMotorInit[0] = true;
    }
    else if (slider[1].motor.CheckID(CAN_RxMsg->ID))
    {
        slider[1].motor.update(CAN_RxMsg->Data);
        sliderMotorInit[1] = true;
    }
}

void SliderControllerClassdef<Motor_C610>::clear()
{
    slider[0].clearCommand();
    slider[1].clearCommand();
    acutate();
}

//////////////////////////////////////////// 6020滑块 //////////////////////////////////////////////////////////
SliderControllerClassdef<Motor_GM6020>::SliderControllerClassdef()
{
    slider[RIGHT_JOINT].positionLoop.SetPIDParam(10, 0, 0, 1000, 3000);
    slider[LEFT_JOINT].positionLoop.SetPIDParam(10, 0, 0, 1000, 3000);

    slider[RIGHT_JOINT].speedLoop.SetPIDParam(100, 500, 0, 3000, 30000);
    slider[LEFT_JOINT].speedLoop.SetPIDParam(100, 500, 0, 3000, 30000);

    slider[RIGHT_JOINT].speedLoop.I_SeparThresh = 50;
    slider[LEFT_JOINT].speedLoop.I_SeparThresh = 50;

    slider[RIGHT_JOINT].setEncoderOffset(600);
    slider[LEFT_JOINT].setEncoderOffset(0);
}

void SliderControllerClassdef<Motor_GM6020>::init()
{
}

void SliderControllerClassdef<Motor_GM6020>::update(float targetPos[2])
{
    for (int i = 0; i < 2; i++)
    {
        slider[i].update(targetPos[i]);
    }
}

void SliderControllerClassdef<Motor_GM6020>::adjust()
{
    for (int i = 0; i < 2; i++)
    {
        slider[i].adjust();
    }
}

void SliderControllerClassdef<Motor_GM6020>::acutate()
{
    if (canTxPort == nullptr)
    {
        return;
    }

    canTxPack = MotorMsgPack(canTxPack, slider[0].motor, slider[1].motor);
    xQueueSend(canTxPort, &(canTxPack.Id2ff), 0);
}

void SliderControllerClassdef<Motor_GM6020>::updateMotorData(CAN_COB *CAN_RxMsg)
{
    if (slider[0].motor.CheckID(CAN_RxMsg->ID))
    {
        slider[0].motor.update(CAN_RxMsg->Data);
        sliderMotorInit[0] = true;
    }
    else if (slider[1].motor.CheckID(CAN_RxMsg->ID))
    {
        slider[1].motor.update(CAN_RxMsg->Data);
        sliderMotorInit[1] = true;
    }
}

void SliderControllerClassdef<Motor_GM6020>::clear()
{
    slider[0].clearCommand();
    slider[1].clearCommand();
    acutate();
}

void SliderControllerClassdef<Motor_GM6020>::setTorqueOut(float _torque[2])
{
    slider[0].setCurrentOut(std_lib::constrain(_torque[0]*30000.f/1.2f,-30000.f,30000.f));
    slider[1].setCurrentOut(std_lib::constrain(_torque[1]*30000.f/1.2f,-30000.f,30000.f));
    acutate();
}
