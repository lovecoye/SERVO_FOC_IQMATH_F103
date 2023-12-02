#ifndef __LOWPASS_FILTER_H__
#define __LOWPASS_FILTER_H__
#include <stdint.h>
#include "IQmathLib.h"

typedef struct _LOWPASS_FILTER_T {
  uint16_t Tf;                 // ��ͨ�˲�ʱ�䳣��
  uint32_t timestamp_prev;  // ��¼�ϴ�ִ�е�ʱ���
	_iq alpha;
  _iq y_prev;             // ��һ��ʱ�䲽�Ĺ���ֵ
} LOWPASS_FILTER_T;

void LOWPASS_FILTER_Init(LOWPASS_FILTER_T* filter, uint16_t time_constant);
_iq LOWPASS_FILTER_Calc(LOWPASS_FILTER_T* filter, _iq x);

#endif