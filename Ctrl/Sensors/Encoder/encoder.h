#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//class Encoder_HandleTypeDef
typedef struct __Encoder_HandleTypeDef
{
    //编码器变量声明
    uint8_t sensor_direction;       // 编码器旋转方向定义
    uint32_t angle_prev_ts; 				// 上次调用 getAngle 的时间戳
    uint32_t vel_angle_prev_ts;			// 最后速度计算时间戳
    int32_t full_rotations;				  // 总圈数计数
    int32_t vel_full_rotations;     // 用于速度计算的先前完整旋转圈数
    float vel_angle_prev;           // 最后一次调用 getVelocity 时的角度
    float angle_prev;               // 最后一次调用 getSensorAngle() 的输出结果，用于得到完整的圈数和速度
	
    //编码器函数指针
    void (*Sensor_init)(struct __Encoder_HandleTypeDef *encoderHandle);
    void (*Sensor_update)(struct __Encoder_HandleTypeDef *encoderHandle);
    float (*getAngle)(struct __Encoder_HandleTypeDef *encoderHandle);
    float (*getVelocity)(struct __Encoder_HandleTypeDef *encoderHandle);
    float (*getMechanicalAngle)(struct __Encoder_HandleTypeDef *encoderHandle);
    float (*getSensorAngle)();
} Encoder_HandleTypeDef;

//编码器构造函数
void encoder_Constructor(Encoder_HandleTypeDef *encoderObj);

//编码器初始化
void encoder_SensorInit(Encoder_HandleTypeDef *encoderHandle);

//
void encoder_SensorUpdate(Encoder_HandleTypeDef *encoderHandle);

//
float encoder_GetAngle(Encoder_HandleTypeDef *encoderHandle);

//
float encoder_GetVelocity(Encoder_HandleTypeDef *encoderHandle);

//
float encoder_GetMechanicalAngle(Encoder_HandleTypeDef *encoderHandle);

//获取编码器角度原始数据
float encoder_GetSensorAngle();

#ifdef __cplusplus
}
#endif
#endif /*__ENCODER_H__ */
