#ifndef __SENSORS_H__
#define __SENSORS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "IQmathLib.h"

//编码器
typedef struct __EncoderTypeDef
{
    int32_t full_rotations; // 总圈数计数
	  int32_t vel_full_rotations; //用于速度计算的先前完整旋转圈数
	  uint32_t angle_prev_ts; // 上次调用 getAngle 的时间戳
    uint32_t vel_angle_prev_ts; // 最后速度计算时间戳
    _iq angle_prev; // 最后一次调用 getSensorAngle() 的输出结果，用于得到完整的圈数和速度
	  _iq vel_angle_prev; // 最后一次调用 getVelocity 时的角度
} EncoderTypeDef;

//输入编码器角度原始数据
void setSensorAngle(EncoderTypeDef *encoder, _iq sensorAngle);
//获取多圈编码器角度值
_iq getAngle(EncoderTypeDef *encoder);
//获取单圈编码器角度值
_iq getMechanicalAngle(EncoderTypeDef *encoder);
//获取速度
_iq getVelocity(EncoderTypeDef *encoder);

void setAnglePrevTs(EncoderTypeDef *encoder,uint32_t _angle_prev_ts);

//电流传感器
typedef struct __CurrSensorTypeDef
{
    _iq currentA;			// A相电流
    _iq currentB;			// B相电流
    _iq currentC;			// C相电流
    _iq offsetIA;			// A相电流零点偏移量
    _iq offsetIB;			// B相电流零点偏移量
    _iq offsetIC;			// C相电流零点偏移量
} CurrSensorTypeDef;

//输入三相电流采样结果
void setPhaseCurrents(CurrSensorTypeDef *CurrSensor, _iq PhaseCurrentA, _iq PhaseCurrentB, _iq PhaseCurrentC);
//查找ADC零点偏移量
void setCurrCaliOffsets(CurrSensorTypeDef *CurrSensor, _iq offsetA, _iq offsetB, _iq offsetC);

#ifdef __cplusplus
}
#endif
#endif /*__SENSORS_H__ */
