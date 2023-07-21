#ifndef _DIFF_CALCULATOR_H_
#define _DIFF_CALCULATOR_H_

#include "SRML.h"

template <int Length>
class DiffCalculator : public myPIDTimer
{
public:
    float diff;

    float calc(float input)
    {
        if (UpdataTimeStamp()) // 计算dt
            return 0;

        return calc(input, this->dt);
    }

    float calc(float input, float _dt)
    {
        float origin_diff;

        origin_diff = (input - last_data) / _dt; // 差分原始数据
        last_data = input;                       // 更新上次数据

        diff = mf.f(origin_diff);
        return diff;
    }

private:
    MeanFilter<Length> mf;
    float last_data = 0;
};

#endif
