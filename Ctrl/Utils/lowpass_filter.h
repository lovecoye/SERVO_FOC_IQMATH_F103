#ifndef __LOWPASS_FILTER_H__
#define __LOWPASS_FILTER_H__
#include <stdint.h>
#include "IQmathLib.h"

typedef struct _LOWPASS_FILTER_T {
  uint16_t Tf;                 // 低通滤波时间常数
  uint32_t timestamp_prev;  // 记录上次执行的时间戳
	_iq alpha;
  _iq y_prev;             // 上一个时间步的过滤值
} LOWPASS_FILTER_T;

void LOWPASS_FILTER_Init(LOWPASS_FILTER_T* filter, uint16_t time_constant);
_iq LOWPASS_FILTER_Calc(LOWPASS_FILTER_T* filter, _iq x);

#endif