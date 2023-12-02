#include "lowpass_filter.h"
#include "pid.h"
#include "sensors.h"
#include "foc_utils.h"
#include "dirverFOC.h"
#include "getmicros.h"

LOWPASS_FILTER_T M0_Vel_Flt;
LOWPASS_FILTER_T M0_Curr_Flt;
LOWPASS_FILTER_T M0_Vout_Flt;

PID_T vel_loop_M0;
PID_T angle_loop_M0;
PID_T current_loop_M0;

EncoderTypeDef MA730;
CurrSensorTypeDef CS_M0;

writePwmFuncTypeDef writePwm;

_iq voltage_power_supply;
_iq zero_electric_angle = 0;
int PP = 1, DIR = 1;

float motor_position, motor_current, motor_velocity, phasevoltage;
//=================PID 设置函数=================
//速度PID
void DFOC_M0_SET_VEL_PID(_iq P, _iq I, _iq D, _iq ramp, _iq limit) //M0角度环PID设置
{
    vel_loop_M0.P = P;
    vel_loop_M0.I = I;
    vel_loop_M0.D = D;
    vel_loop_M0.output_ramp = ramp;
    vel_loop_M0.limit = limit;
}
//角度PID
void DFOC_M0_SET_ANGLE_PID(_iq P, _iq I, _iq D, _iq ramp, _iq limit) //M0角度环PID设置
{
    angle_loop_M0.P = P;
    angle_loop_M0.I = I;
    angle_loop_M0.D = D;
    angle_loop_M0.output_ramp = ramp;
    angle_loop_M0.limit = limit;
}
//电流PID
void DFOC_M0_SET_CURRENT_PID(_iq P, _iq I, _iq D, _iq ramp) //M0电流环PID设置
{
    current_loop_M0.P = P;
    current_loop_M0.I = I;
    current_loop_M0.D = D;
    current_loop_M0.output_ramp = ramp;
}

_iq _normalizeAngle(_iq angle)
{
    _iq a = angle % 205887; //2PI
    return a >= 0 ? a : (a + 205887);//2PI
}

//M0速度PID接口
_iq DFOC_M0_VEL_PID(_iq error)
{
    return PID_Calc(&vel_loop_M0, error);
}
//M0角度PID接口
_iq DFOC_M0_ANGLE_PID(_iq error)
{
    return PID_Calc(&angle_loop_M0, error);
}

//PWM驱动注册
void writePwmRegister(writePwmFuncTypeDef writePwmFunc)
{
    writePwm = writePwmFunc;
}

// 设置PWM到控制器输出
void setPwm(_iq _Ua, _iq _Ub, _iq _Uc)
{
    // 计算占空比
    // 限制占空比从0到1
    _iq dc_a = _IQdiv(_Ua, voltage_power_supply);
    _iq dc_b = _IQdiv(_Ub, voltage_power_supply);
    _iq dc_c = _IQdiv(_Uc, voltage_power_supply);

    //写入PWM到PWM 0 1 2 通道
    writePwm(dc_a, dc_b, dc_c);
}

void setTorqueSPWM(_iq Uq, _iq angle_el)
{
    angle_el = _normalizeAngle(angle_el);
    // 帕克逆变换
    _iq Ualpha = _IQmpy(-Uq, _IQsin(angle_el));
    _iq Ubeta = _IQmpy(Uq, _IQcos(angle_el));

    // 克拉克逆变换
    _iq Ua = Ualpha + _IQdiv(voltage_power_supply, 65536);//2
    _iq Ub = _IQdiv((_IQmpy(_IQsqrt(98304), Ubeta) - Ualpha), 65536) + _IQdiv(voltage_power_supply, 65536);
    _iq Uc = _IQdiv(-Ualpha - _IQmpy(_IQsqrt(98304), Ubeta), 65536) + _IQdiv(voltage_power_supply, 65536);

//		_iq uout = LOWPASS_FILTER_Calc(&M0_Vout_Flt, Ua);
		phasevoltage=_IQtoF(Ua);

    setPwm(Ua, Ub, Uc);
}

void setTorque(_iq Uq, _iq angle_el)
{
    uint8_t sector;
    _iq Uout;

    Uout = _IQdiv(Uq, voltage_power_supply);
    angle_el = _normalizeAngle(angle_el + 51471);//_PI_2

    sector = _IQint(_IQdiv(angle_el, 34314)) + 1; //_PI_3

    _iq T1 = _IQmpy(_IQmpy(56755, _IQsin(_IQmpyI32(34314, sector) - angle_el)), Uout); //_SQRT3 _PI_3
    _iq T2 = _IQmpy(_IQmpy(56755, _IQsin(angle_el - _IQmpyI32(34314, sector - 1))), Uout); //_SQRT3 _PI_3
    _iq T0 = 32768 - T1 - T2;//1
    _iq T0div2 = _IQdiv(T0, 65536); //2

    // 计算占空比
    _iq Ta, Tb, Tc;
    switch (sector)
    {
    case 1:
        Ta = T1 + T2 + T0div2;
        Tb = T2 + T0div2;
        Tc = T0div2;
        break;
    case 2:
        Ta = T1 + T0div2;
        Tb = T1 + T2 + T0div2;
        Tc = T0div2;
        break;
    case 3:
        Ta = T0div2;
        Tb = T1 + T2 + T0div2;
        Tc = T2 + T0div2;
        break;
    case 4:
        Ta = T0div2;
        Tb = T1 + T0div2;
        Tc = T1 + T2 + T0div2;
        break;
    case 5:
        Ta = T2 + T0div2;
        Tb = T0div2;
        Tc = T1 + T2 + T0div2;
        break;
    case 6:
        Ta = T1 + T2 + T0div2;
        Tb = T0div2;
        Tc = T1 + T0div2;
        break;
    default:
        // 错误状态
        Ta = 0;
        Tb = 0;
        Tc = 0;
    }

    // 计算相位电压和中心
    _iq Ua = _IQmpy(Ta, voltage_power_supply);
    _iq Ub = _IQmpy(Tb, voltage_power_supply);
    _iq Uc = _IQmpy(Tc, voltage_power_supply);
		
		phasevoltage=_IQtoF(Ua);

    setPwm(Ua, Ub, Uc);
}

void DFOC_Vbus(_iq power_supply)
{
    voltage_power_supply = power_supply;

    //PID 加载
    PID_Init(&vel_loop_M0, _IQ(2), 0, 0, _IQ(65535), _IQ(5));
    PID_Init(&angle_loop_M0, _IQ(2), 0, 0, _IQ(65535), _IQ(100));
    PID_Init(&current_loop_M0, _IQ(1.2), 0, 0, _IQ(65535), _IQ(5));

    //低通滤波初始化
    LOWPASS_FILTER_Init(&M0_Vel_Flt, 65000);			// Tf = 65ms   //M0速度环
    LOWPASS_FILTER_Init(&M0_Curr_Flt, 3000);		// Tf = 3ms   //M0电流环
    LOWPASS_FILTER_Init(&M0_Vout_Flt, 500);		// Tf = 3ms   //M0电流环
}

_iq _electricalAngle()
{
    return  _normalizeAngle(_IQmpyI32(getMechanicalAngle(&MA730), (DIR *  PP)) - zero_electric_angle);
}

void DFOC_alignSensor(int _PP, int _DIR)
{
    PP = _PP;
    DIR = _DIR;
    setTorqueSPWM(_IQ(2), _IQ(_3PI_2));  //起劲
    delay(2000);
    zero_electric_angle = _electricalAngle();
    setTorqueSPWM(0, _IQ(_3PI_2));  //松劲（解除校准）
}

_iq DFOC_M0_Angle()
{
    _iq position = _IQmpyI32(getAngle(&MA730), DIR);
    motor_position = _IQtoF(position);
    return position;
}

void setCurrents(_iq _currA, _iq _currB, _iq _currC)
{
    setPhaseCurrents(&CS_M0, _currA, _currB, _currC);
}

void setPrevTs(uint32_t __angle_prev_ts)
{
    setAnglePrevTs(&MA730, __angle_prev_ts);
}

void setCurrentsOffsets(_iq _offsetA, _iq _offsetB, _iq _offsetC)
{
    setCurrCaliOffsets(&CS_M0, _offsetA, _offsetB, _offsetC);
}

void setAngle(_iq _angle)
{
    setSensorAngle(&MA730, _angle);
}

//=========================电流读取=========================
//通过Ia,Ib,Ic计算Iq,Id(目前仅输出Iq)
_iq cal_Iq_Id(_iq current_a, _iq current_b, _iq angle_el)
{
    _iq I_alpha = current_a;
    _iq I_beta = _IQmpy(18918, current_a) + _IQmpy(37837, current_b);//0.577350259 1.15470052

    _iq ct = _IQcos(angle_el);
    _iq st = _IQsin(angle_el);
    //float I_d = I_alpha * ct + I_beta * st;
    _iq I_q = _IQmpy(I_beta, ct) - _IQmpy(I_alpha, st);
    return I_q;
}
_iq DFOC_M0_Current()
{
    _iq I_q_M0_ori = cal_Iq_Id(CS_M0.currentA, CS_M0.currentB, _electricalAngle());
    _iq I_q_M0_flit = LOWPASS_FILTER_Calc(&M0_Curr_Flt, I_q_M0_ori);
    motor_current = _IQtoF(I_q_M0_flit);
    return I_q_M0_flit;
}

_iq DFOC_M0_Velocity()
{
    //获取速度数据并滤波
    _iq vel_M0_ori = getVelocity(&MA730);
    _iq vel_M0_flit = LOWPASS_FILTER_Calc(&M0_Vel_Flt, _IQmpyI32(vel_M0_ori, DIR));
    motor_velocity = _IQtoF(vel_M0_flit);
    return vel_M0_flit;   //考虑方向
}

//================简易接口函数================
void DFOC_M0_setTorque(_iq Target)            //电流力矩环
{
    setTorque(PID_Calc(&current_loop_M0, DFOC_M0_Current() - Target), _electricalAngle());
}

void DFOC_M0_set_Velocity_Angle(_iq Target)   //角度-速度-力 位置闭环
{
    //setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI)),_electricalAngle());        //改进前
    DFOC_M0_setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID(_IQmpy((Target - DFOC_M0_Angle()), 1877468)) - DFOC_M0_Velocity())); //改进后
}

void DFOC_M0_setVelocity(_iq Target)          //速度闭环
{
    //setTorque(DFOC_M0_VEL_PID((Target-DFOC_M0_Velocity())*180/PI),_electricalAngle());   //改进前
    DFOC_M0_setTorque(DFOC_M0_VEL_PID(_IQmpy((Target - DFOC_M0_Velocity()), 1877468)));        //改进后 rad/s
}

void DFOC_M0_set_Force_Angle(_iq Target)      //力位闭环
{
    //setTorque(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI),_electricalAngle());   //改进前
    DFOC_M0_setTorque(DFOC_M0_ANGLE_PID(_IQmpy((Target - DFOC_M0_Angle()), 1877468))); //改进后 180/PI=57.295779513
}
