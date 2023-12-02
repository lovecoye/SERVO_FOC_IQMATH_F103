#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>
#include "IQmathLib.h"

typedef struct _PID_T
{
    _iq P;            // 比例增益
    _iq I;            // 积分增益
    _iq D;            // 微分增益
    _iq output_ramp;  // 输出值的最大变化速度
    _iq limit;        // 最大输出值

    _iq error_prev;         // 跟踪最后一次误差值
    _iq output_prev;        // 最后一次PID输出值
    _iq integral_prev;      // 最后一次积分值
    uint32_t timestamp_prev;  // 记录最后一次时间戳
} PID_T;

void PID_Init(PID_T *pid, _iq P, _iq I, _iq D, _iq ramp, _iq limit);
_iq PID_Calc(PID_T *pid, _iq error);
void PID_Reset(PID_T *pid);

#endif