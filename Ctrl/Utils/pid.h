#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>
#include "IQmathLib.h"

typedef struct _PID_T
{
    _iq P;            // ��������
    _iq I;            // ��������
    _iq D;            // ΢������
    _iq output_ramp;  // ���ֵ�����仯�ٶ�
    _iq limit;        // ������ֵ

    _iq error_prev;         // �������һ�����ֵ
    _iq output_prev;        // ���һ��PID���ֵ
    _iq integral_prev;      // ���һ�λ���ֵ
    uint32_t timestamp_prev;  // ��¼���һ��ʱ���
} PID_T;

void PID_Init(PID_T *pid, _iq P, _iq I, _iq D, _iq ramp, _iq limit);
_iq PID_Calc(PID_T *pid, _iq error);
void PID_Reset(PID_T *pid);

#endif