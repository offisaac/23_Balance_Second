#ifndef _SLIDING_BLOCK_CONTROLLER_H_
#define _SLIDING_BLOCK_CONTROLLER_H_

#include "Slider.h"
#include "FreeRTOS.h"
#include "queue.h"

template <class motorType>
class SliderControllerClassdef
{
private:
public:
};

template <>
class SliderControllerClassdef<Motor_C610>
{
private:
    QueueHandle_t canTxPort = nullptr;
    Motor_CAN_COB canTxPack;
    bool sliderMotorInit[2] = {false, false};

public:
    SliderClassdef<Motor_C610> slider[2] = {SliderClassdef<Motor_C610>(RIGHT_SLIDER_ID, -1, 1, 36.f), SliderClassdef<Motor_C610>(LEFT_SLIDER_ID, 1, 1, 36.f)};

    SliderControllerClassdef();
    void importQueueHander(QueueHandle_t _canTxPort) { canTxPort = _canTxPort; }
    void updateMotorData(CAN_COB *CAN_RxMsg);

    void init();
    void update(float targetPos[2]);
    void adjust();
    void acutate();
    void clear();
};

template <>
class SliderControllerClassdef<Motor_GM6020>
{
private:
    QueueHandle_t canTxPort = nullptr;
    Motor_CAN_COB canTxPack;
    bool sliderMotorInit[2] = {false, false};

public:
    SliderClassdef<Motor_GM6020> slider[2] = {SliderClassdef<Motor_GM6020>(RIGHT_SLIDER_ID, -1, 1, 1.f), SliderClassdef<Motor_GM6020>(LEFT_SLIDER_ID, 1, 1, 1.f)};

    SliderControllerClassdef();
    void importQueueHander(QueueHandle_t _canTxPort) { canTxPort = _canTxPort; }
    void updateMotorData(CAN_COB *CAN_RxMsg);

    void init();
    void update(float targetPos[2]);
    void adjust();
    void acutate();
    void clear();
};

#endif
