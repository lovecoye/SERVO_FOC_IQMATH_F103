#ifndef __DIRVERFOC_H__
#define __DIRVERFOC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "IQmathLib.h"

extern float motor_position, motor_current, motor_velocity,phasevoltage;


void setPwm(_iq Ua, _iq Ub, _iq Uc);
void setTorque(_iq Uq, _iq angle_el);
void setTorqueSPWM(_iq Uq, _iq angle_el);
_iq _normalizeAngle(_iq angle);
void DFOC_Vbus(_iq power_supply);
void DFOC_alignSensor(int _PP, int _DIR);
_iq _electricalAngle();

//传感器读取
_iq DFOC_M0_Velocity();
_iq DFOC_M0_Angle();
_iq DFOC_M0_Current();

//PID
void DFOC_M0_SET_ANGLE_PID(_iq P, _iq I, _iq D, _iq ramp, _iq limit);
void DFOC_M0_SET_VEL_PID(_iq P, _iq I, _iq D, _iq ramp, _iq limit);
void DFOC_M0_SET_CURRENT_PID(_iq P, _iq I, _iq D, _iq ramp);
_iq DFOC_M0_VEL_PID(_iq error);
_iq DFOC_M0_ANGLE_PID(_iq error);
//接口函数
void DFOC_M0_set_Velocity_Angle(_iq Target);
void DFOC_M0_setVelocity(_iq Target);
void DFOC_M0_set_Force_Angle(_iq Target);
void DFOC_M0_setTorque(_iq Target);

typedef void (*writePwmFuncTypeDef)(_iq UaPer, _iq UbPer, _iq UcPer);
void writePwmRegister(writePwmFuncTypeDef writePwmFunc);

void setCurrents(_iq _currA,_iq _currB,_iq _currC);
void setCurrentsOffsets(_iq _offsetA, _iq _offsetB, _iq _offsetC);
void setAngle(_iq _angle);
void setPrevTs(uint32_t __angle_prev_ts);

#ifdef __cplusplus
}
#endif
#endif /*__DIRVERFOC_H__ */
