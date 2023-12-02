#include "ma730.h"
#include "spi.h"
#include "math.h"

//int32_t full_rotations = 0; // full rotation tracking;
//float angle_prev = 0;

float getSensorAngle()
{
    uint16_t rxBuf_SPI;
    uint16_t txBuf_SPI = 0x00;
    uint16_t rawAngle;
    GPIOA->BRR = GPIO_PIN_15;
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&txBuf_SPI, (uint8_t *)&rxBuf_SPI, 1, HAL_MAX_DELAY);
    GPIOA->BSRR = GPIO_PIN_15;
    rawAngle = (rxBuf_SPI & 0xFFFC) >> 2;
    return rawAngle*0.00038349519697; //1/14bit�ֱ���*2PI=0.00038349519697 ���ر���������
}


//float getAngle()
//{
//    float val = getAngle_Without_track();
//    float d_angle = val - angle_prev;
//    //������ת����Ȧ��
//    //ͨ���жϽǶȱ仯�Ƿ����80%��һȦ(0.8f*6.28318530718f)���ж��Ƿ������������������ˣ���full_rotations����1�����d_angleС��0�������1�����d_angle����0����
//    if(fabs(d_angle) > (0.8f * 6.28318530718f)) full_rotations += ( d_angle > 0 ) ? -1 : 1;
//    angle_prev = val;
//    return (float)full_rotations * 6.28318530718f + angle_prev;
//}