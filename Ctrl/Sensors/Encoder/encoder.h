#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//class Encoder_HandleTypeDef
typedef struct __Encoder_HandleTypeDef
{
    //��������������
    uint8_t sensor_direction;       // ��������ת������
    uint32_t angle_prev_ts; 				// �ϴε��� getAngle ��ʱ���
    uint32_t vel_angle_prev_ts;			// ����ٶȼ���ʱ���
    int32_t full_rotations;				  // ��Ȧ������
    int32_t vel_full_rotations;     // �����ٶȼ������ǰ������תȦ��
    float vel_angle_prev;           // ���һ�ε��� getVelocity ʱ�ĽǶ�
    float angle_prev;               // ���һ�ε��� getSensorAngle() �������������ڵõ�������Ȧ�����ٶ�
	
    //����������ָ��
    void (*Sensor_init)(struct __Encoder_HandleTypeDef *encoderHandle);
    void (*Sensor_update)(struct __Encoder_HandleTypeDef *encoderHandle);
    float (*getAngle)(struct __Encoder_HandleTypeDef *encoderHandle);
    float (*getVelocity)(struct __Encoder_HandleTypeDef *encoderHandle);
    float (*getMechanicalAngle)(struct __Encoder_HandleTypeDef *encoderHandle);
    float (*getSensorAngle)();
} Encoder_HandleTypeDef;

//���������캯��
void encoder_Constructor(Encoder_HandleTypeDef *encoderObj);

//��������ʼ��
void encoder_SensorInit(Encoder_HandleTypeDef *encoderHandle);

//
void encoder_SensorUpdate(Encoder_HandleTypeDef *encoderHandle);

//
float encoder_GetAngle(Encoder_HandleTypeDef *encoderHandle);

//
float encoder_GetVelocity(Encoder_HandleTypeDef *encoderHandle);

//
float encoder_GetMechanicalAngle(Encoder_HandleTypeDef *encoderHandle);

//��ȡ�������Ƕ�ԭʼ����
float encoder_GetSensorAngle();

#ifdef __cplusplus
}
#endif
#endif /*__ENCODER_H__ */
