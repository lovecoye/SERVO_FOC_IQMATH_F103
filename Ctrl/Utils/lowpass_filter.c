#include "lowpass_filter.h"
#include "getmicros.h"

void LOWPASS_FILTER_Init(LOWPASS_FILTER_T *filter, uint16_t time_constant)
{
    filter->Tf = time_constant;
    filter->y_prev = 0;
//    filter->timestamp_prev = micros();
    filter->alpha = _IQdiv( _IQ(filter->Tf), _IQ(filter->Tf + 100));

}

_iq LOWPASS_FILTER_Calc(LOWPASS_FILTER_T *filter, _iq x)
{
//    uint32_t timestamp = micros();
//    uint32_t dt = timestamp - filter->timestamp_prev;

//    if (dt < 0)
//        dt = 80;
//    else if (dt > 300000)
//    {
//        // 如果大于300ms则不处理
//        filter->y_prev = x;
//        filter->timestamp_prev = timestamp;
//        return x;
//    }
	
//    _iq alpha = _IQdiv( _IQ(filter->Tf), _IQ(filter->Tf + dt));
    _iq y = _IQmpy(filter->alpha, filter->y_prev) + _IQmpy((32768 - filter->alpha), x);//1
    filter->y_prev = y;
//    filter->timestamp_prev = timestamp;
    return y;
}