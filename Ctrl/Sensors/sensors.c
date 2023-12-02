#include "sensors.h"
#include "foc_utils.h"

void setSensorAngle(EncoderTypeDef *encoder, _iq sensorAngle)
{
    _iq d_angle = sensorAngle - encoder->angle_prev;
    if(_IQabs(d_angle) > 163840)//2PI*0.8≈5
        encoder->full_rotations += ( d_angle > 0 ) ? -1 : 1;    // 圈数检测
    encoder->angle_prev = sensorAngle;
}

void setAnglePrevTs(EncoderTypeDef *encoder, uint32_t _angle_prev_ts)
{
    encoder->angle_prev_ts = _angle_prev_ts;
}

_iq getMechanicalAngle(EncoderTypeDef *encoder)
{
    return encoder->angle_prev;
}

_iq getAngle(EncoderTypeDef *encoder)
{
    return _IQmpyI32(205887, encoder->full_rotations) + encoder->angle_prev;//2PI
}

_iq getVelocity(EncoderTypeDef *encoder)
{
    // 计算采样时间
//    uint32_t _Ts = encoder->angle_prev_ts - encoder->vel_angle_prev_ts;
//    if (_Ts <= 0 || _Ts > 65535) _Ts = 80;
//    _iq Ts = _IQdiv(_IQ(_Ts), 32768000); //以毫秒的方式存储，防止计算越界 1000
    // 速度计算
    _iq vel = _IQmpyI32(_IQdiv((_IQmpyI32(205887, (encoder->full_rotations - encoder->vel_full_rotations)) + (encoder->angle_prev - encoder->vel_angle_prev)), 3276), 1000);
    // 保存变量以待将来使用
    encoder->vel_angle_prev = encoder->angle_prev;
    encoder->vel_full_rotations = encoder->full_rotations;
//    encoder->vel_angle_prev_ts = encoder->angle_prev_ts;
    return vel;
}

void setPhaseCurrents(CurrSensorTypeDef *CurrSensor, _iq PhaseCurrentA, _iq PhaseCurrentB, _iq PhaseCurrentC)
{
    CurrSensor->currentA = PhaseCurrentA - CurrSensor->offsetIA;
    CurrSensor->currentB = PhaseCurrentB - CurrSensor->offsetIB;
    CurrSensor->currentC = PhaseCurrentC - CurrSensor->offsetIC;
}

void setCurrCaliOffsets(CurrSensorTypeDef *CurrSensor, _iq offsetA, _iq offsetB, _iq offsetC)
{
    CurrSensor->offsetIA = offsetA;
    CurrSensor->offsetIB = offsetB;
    CurrSensor->offsetIC = offsetC;
}
