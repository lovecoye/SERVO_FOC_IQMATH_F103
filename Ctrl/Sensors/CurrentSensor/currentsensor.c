#include "currentsensor.h"

#define _ADC_VOLTAGE 3.3f
#define _ADC_RESOLUTION 4095.0f

// ADC ��������ѹת���������
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

#define NOT_SET -12345.0
#define _isset(a) ( (a) != (NOT_SET) )

float CurrSense::readADCVoltageInline(const int pinA){
  uint32_t raw_adc = analogRead(pinA);
  return raw_adc * _ADC_CONV;
}
void CurrSense::configureADCInline(const int pinA,const int pinB, const int pinC){
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);
}

// ���� ADC ��ƫ�����ĺ���
void CurrSense::calibrateOffsets(){
    const int calibration_rounds = 1000;

    // ����0����ʱ��ĵ�ѹ
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    // ����1000��
    for (int i = 0; i < calibration_rounds; i++) {
        offset_ia += readADCVoltageInline(pinA);
        offset_ib += readADCVoltageInline(pinB);
        if(_isset(pinC)) offset_ic += readADCVoltageInline(pinC);
        delay(1);
    }
    // ��ƽ�����õ����
    offset_ia = offset_ia / calibration_rounds;
    offset_ib = offset_ib / calibration_rounds;
    if(_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}

void CurrSense::init(){
    // ���ú���
    configureADCInline(pinA,pinB,pinC);
    // У׼
    calibrateOffsets();
}


// ��ȡȫ���������

void CurrSense::getPhaseCurrents(){
    current_a = (readADCVoltageInline(pinA) - offset_ia)*gain_a;// amps
    current_b = (readADCVoltageInline(pinB) - offset_ib)*gain_b;// amps
    current_c = (!_isset(pinC)) ? 0 : (readADCVoltageInline(pinC) - offset_ic)*gain_c; // amps
}
