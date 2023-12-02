#include "encoder.h"
#include "gpio.h"
#include "spi.h"
#include "getmicros.h"
#include "math.h"

#define _2PI 6.28318530718f

void encoder_Constructor(Encoder_HandleTypeDef *encoderObj)
{
    encoderObj->angle_prev = 0; // 最后一次调用 getSensorAngle() 的输出结果，用于得到完整的圈数和速度
    encoderObj->angle_prev_ts = 0; // 上次调用 getAngle 的时间戳
    encoderObj->vel_angle_prev = 0; // 最后一次调用 getVelocity 时的角度
    encoderObj->vel_angle_prev_ts = 0; // 最后速度计算时间戳
    encoderObj->full_rotations = 0; // 总圈数计数
    encoderObj->vel_full_rotations = 0; //用于速度计算的先前完整旋转圈数
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
    return rawAngle * 0.00038349519697; //1/14bit分辨率*2PI=0.00038349519697 返回编码器弧度
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
    // 圈数检测
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
    // 计算采样时间
    float Ts = (encoderHandle->angle_prev_ts - encoderHandle->vel_angle_prev_ts)*1e-6;
    // 快速修复奇怪的情况（微溢出）
    if(Ts <= 0) Ts = 1e-3f;
    // 速度计算
    float vel = ( (float)(encoderHandle->full_rotations - encoderHandle->vel_full_rotations)*_2PI + (encoderHandle->angle_prev - encoderHandle->vel_angle_prev) ) / Ts;    
    // 保存变量以待将来使用
    encoderHandle->vel_angle_prev = encoderHandle->angle_prev;
    encoderHandle->vel_full_rotations = encoderHandle->full_rotations;
    encoderHandle->vel_angle_prev_ts = encoderHandle->angle_prev_ts;
    return vel;
}