#ifndef __SENSORS_H__
#define __SENSORS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "IQmathLib.h"

//������
typedef struct __EncoderTypeDef
{
    int32_t full_rotations; // ��Ȧ������
	  int32_t vel_full_rotations; //�����ٶȼ������ǰ������תȦ��
	  uint32_t angle_prev_ts; // �ϴε��� getAngle ��ʱ���
    uint32_t vel_angle_prev_ts; // ����ٶȼ���ʱ���
    _iq angle_prev; // ���һ�ε��� getSensorAngle() �������������ڵõ�������Ȧ�����ٶ�
	  _iq vel_angle_prev; // ���һ�ε��� getVelocity ʱ�ĽǶ�
} EncoderTypeDef;

//����������Ƕ�ԭʼ����
void setSensorAngle(EncoderTypeDef *encoder, _iq sensorAngle);
//��ȡ��Ȧ�������Ƕ�ֵ
_iq getAngle(EncoderTypeDef *encoder);
//��ȡ��Ȧ�������Ƕ�ֵ
_iq getMechanicalAngle(EncoderTypeDef *encoder);
//��ȡ�ٶ�
_iq getVelocity(EncoderTypeDef *encoder);

void setAnglePrevTs(EncoderTypeDef *encoder,uint32_t _angle_prev_ts);

//����������
typedef struct __CurrSensorTypeDef
{
    _iq currentA;			// A�����
    _iq currentB;			// B�����
    _iq currentC;			// C�����
    _iq offsetIA;			// A��������ƫ����
    _iq offsetIB;			// B��������ƫ����
    _iq offsetIC;			// C��������ƫ����
} CurrSensorTypeDef;

//������������������
void setPhaseCurrents(CurrSensorTypeDef *CurrSensor, _iq PhaseCurrentA, _iq PhaseCurrentB, _iq PhaseCurrentC);
//����ADC���ƫ����
void setCurrCaliOffsets(CurrSensorTypeDef *CurrSensor, _iq offsetA, _iq offsetB, _iq offsetC);

#ifdef __cplusplus
}
#endif
#endif /*__SENSORS_H__ */
