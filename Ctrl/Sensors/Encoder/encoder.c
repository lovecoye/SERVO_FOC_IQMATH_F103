#include "encoder.h"
#include "gpio.h"
#include "spi.h"
#include "getmicros.h"
#include "math.h"

#define _2PI 6.28318530718f

void encoder_Constructor(Encoder_HandleTypeDef *encoderObj)
{
    encoderObj->angle_prev = 0; // ���һ�ε��� getSensorAngle() �������������ڵõ�������Ȧ�����ٶ�
    encoderObj->angle_prev_ts = 0; // �ϴε��� getAngle ��ʱ���
    encoderObj->vel_angle_prev = 0; // ���һ�ε��� getVelocity ʱ�ĽǶ�
    encoderObj->vel_angle_prev_ts = 0; // ����ٶȼ���ʱ���
    encoderObj->full_rotations = 0; // ��Ȧ������
    encoderObj->vel_full_rotations = 0; //�����ٶȼ������ǰ������תȦ��
    encoderObj->Sensor_init = encoder_SensorInit;
    encoderObj->Sensor_update = encoder_SensorUpdate;
    encoderObj->getAngle = encoder_GetAngle;
    encoderObj->getVelocity = encoder_GetVelocity;
    encoderObj->getMechanicalAngle = encoder_GetMechanicalAngle;
    encoderObj->getSensorAngle = encoder_GetSensorAngle;
}

float encoder_GetSensorAngle()
{
    uint16_t rxBuf_SPI;
    uint16_t txBuf_SPI = 0x00;
    uint16_t rawAngle;
    GPIOA->BRR = GPIO_PIN_15;
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&txBuf_SPI, (uint8_t *)&rxBuf_SPI, 1, HAL_MAX_DELAY);
    GPIOA->BSRR = GPIO_PIN_15;
    rawAngle = (rxBuf_SPI & 0xFFFC) >> 2;
    return rawAngle * 0.00038349519697; //1/14bit�ֱ���*2PI=0.00038349519697 ���ر���������
}

void encoder_SensorInit(Encoder_HandleTypeDef *encoderHandle)
{
    encoderHandle->vel_angle_prev = encoderHandle->getSensorAngle();
    encoderHandle->vel_angle_prev_ts = micros();
    HAL_Delay(1);
    encoderHandle->angle_prev = encoderHandle->getSensorAngle();
    encoderHandle->angle_prev_ts = micros();
}

void encoder_SensorUpdate(Encoder_HandleTypeDef *encoderHandle)
{
    float val = encoderHandle->getSensorAngle();
    encoderHandle->angle_prev_ts = micros();
    float d_angle = val - encoderHandle->angle_prev;
    // Ȧ�����
    if(fabs(d_angle) > (0.8f * _2PI) ) encoderHandle->full_rotations += ( d_angle > 0 ) ? -1 : 1;
    encoderHandle->angle_prev = val;
}

float encoder_GetMechanicalAngle(Encoder_HandleTypeDef *encoderHandle) {
    return encoderHandle->angle_prev;
}

float encoder_GetAngle(Encoder_HandleTypeDef *encoderHandle){
    return (float)encoderHandle->full_rotations * _2PI + encoderHandle->angle_prev;
}

float encoder_GetVelocity(Encoder_HandleTypeDef *encoderHandle) {
    // �������ʱ��
    float Ts = (encoderHandle->angle_prev_ts - encoderHandle->vel_angle_prev_ts)*1e-6;
    // �����޸���ֵ������΢�����
    if(Ts <= 0) Ts = 1e-3f;
    // �ٶȼ���
    float vel = ( (float)(encoderHandle->full_rotations - encoderHandle->vel_full_rotations)*_2PI + (encoderHandle->angle_prev - encoderHandle->vel_angle_prev) ) / Ts;    
    // ��������Դ�����ʹ��
    encoderHandle->vel_angle_prev = encoderHandle->angle_prev;
    encoderHandle->vel_full_rotations = encoderHandle->full_rotations;
    encoderHandle->vel_angle_prev_ts = encoderHandle->angle_prev_ts;
    return vel;
}