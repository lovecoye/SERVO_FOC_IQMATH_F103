#ifndef __CURRENTSENSOR_H__
#define __CURRENTSENSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct __CurrSense
{
    float (*readADCVoltageInline)(const int pinA);
    void (*configureADCInline)(const int pinA,const int pinB, const int pinC);
    void (*calibrateOffsets)();
    void (*init)();
    void (*getPhaseCurrents)();
    float current_a,current_b,current_c;
    int pinA;
    int pinB;
    int pinC;
    float offset_ia;
    float offset_ib;
    float offset_ic;
    float _shunt_resistor;
    float amp_gain;
    
    float volts_to_amps_ratio;
    
    float gain_a;
    float gain_b;
    float gain_c;
	
    int _Mot_Num;
} CurrSense;


#ifdef __cplusplus
}
#endif
#endif /*__CURRENTSENSOR_H__ */
