#ifndef _SLIDING_BLOCK_CONTROLLER_H_
#define _SLIDING_BLOCK_CONTROLLER_H_

#include "Slider.h"
#include "FreeRTOS.h"
#include "queue.h"

class SliderControllerClassdef
{
private:
    QueueHandle_t canTxPort = nullptr;
    Motor_CAN_COB canTxPack;
    bool sliderMotorInit[2] = {false,false};

public:
    SliderClassdef slider[2] = {SliderClassdef(RIGHT_SLIDER_ID, -1, 1), SliderClassdef(LEFT_SLIDER_ID, 1, 1)};

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
