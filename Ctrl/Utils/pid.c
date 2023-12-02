#include "pid.h"
#include "getmicros.h"
#include "foc_utils.h"

void PID_Init(PID_T *pid, _iq P, _iq I, _iq D, _iq ramp, _iq limit)
{
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->limit = limit;
    pid->error_prev = 0;
    pid->output_prev = 0;
    pid->integral_prev = 0;
    pid->timestamp_prev = micros();
}

_iq PID_Calc(PID_T *pid, _iq error)
{
//    uint32_t timestamp_now = micros();
//    uint32_t _Ts = timestamp_now - pid->timestamp_prev;
//    if (_Ts <= 0 || _Ts > 65535) _Ts = 80;
//    _iq Ts = _IQdiv(_IQ(_Ts), 32768000); //�Ժ���ķ�ʽ�洢����ֹ����Խ�� 1000

    _iq proportional = _IQmpy(pid->P, error);
	  //ԭʽfloat integral = integral_prev + I*Ts*0.5f*(error + error_prev);���ｫTS�Ŵ�10��������ʱ��i��С10������
    _iq integral = pid->integral_prev + _IQmpy(_IQmpy( pid->I, 16),(error + pid->error_prev));
    integral = _constrain(integral, -pid->limit, pid->limit);
    _iq derivative = _IQdiv( _IQmpy(pid->D, (error - pid->error_prev)), 3276);//derivative��С1000��

    _iq output = proportional + integral + derivative;

    output = _constrain(output, -pid->limit, pid->limit);

    if (pid->output_ramp > 0)
    {
        _iq output_rate =_IQdiv((output - pid->output_prev), 3276);//�˴��仯��ʱ�䵥λms
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + _IQmpy(pid->output_ramp, 3276);
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev -_IQmpy(pid->output_ramp, 3276);
    }

    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
//    pid->timestamp_prev = timestamp_now;
    return output;
}

void PID_Reset(PID_T *pid)
{
    pid->integral_prev = 0;
    pid->output_prev = 0;
    pid->error_prev = 0;
}