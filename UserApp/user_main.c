#include "main.h"
#include "spi.h"
#include "i2c.h"
#include "gpio.h"
#include "tim.h"
#include "can.h"
#include "user_main.h"
#include "dirverFOC.h"
#include "adc.h"
#include "getmicros.h"
#include "IQmathLib.h"

#define _POLE_PAIRS 2//极对数
#define _DIR -1//转动方向
#define ADC_MAX_NUM 3
#define _COUNTER_PERIOD_TIM1 3600//PWM计数器
#define AS5600_ADDR (0x36<<1)

void spi_transmit_receive(uint16_t data_in, uint16_t *data_out);
uint16_t bsp_as5600GetRawAngle(void);
void funcWritePwm(_iq UaPer, _iq UbPer, _iq UcPer);
_iq getRawAngle();

uint8_t setFlag;
uint16_t outputEncoder;
uint16_t ADC_Values[ADC_MAX_NUM] = {0};

float vel_p = 0.025, vel_i = 0.08, vel_d = 0.0005, vel_limit = 2;
float ang_p = 1, ang_i = 0.5, ang_d = 0.5, ang_limit = 150;
float cur_p = 30, cur_i = 8, cur_d = 0.1;

float cur_target = 1;
float vel_target = 10;
float pos_target = 0;

//float soaBuf[2048];
//uint16_t soaBufFlag;
//uint32_t timein, timeout;

void User_Main(void)
{
    LL_SPI_Enable(SPI1);
    //ADC校准
    HAL_ADCEx_Calibration_Start(&hadc1);

    //采样偏置校准
    HAL_GPIO_WritePin(CAL_GPIO_Port, CAL_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(CAL_GPIO_Port, CAL_Pin, GPIO_PIN_RESET);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

    //注册修改PWM占空比函数
    writePwmRegister(funcWritePwm);

    DFOC_Vbus(_IQ(24));
    DFOC_alignSensor(_POLE_PAIRS, _DIR);

    DFOC_M0_SET_ANGLE_PID(_IQ(ang_p), _IQ(ang_i), _IQ(ang_d), _IQ(65535), _IQ(ang_limit));
    DFOC_M0_SET_CURRENT_PID(_IQ(cur_p), _IQ(cur_i), _IQ(cur_d), _IQ(65535));
    DFOC_M0_SET_VEL_PID(_IQ(vel_p), _IQ(vel_i), _IQ(vel_d), _IQ(65535), _IQ(vel_limit));

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 3200);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
    HAL_ADCEx_InjectedStart(&hadc1);

    while(1)
    {
        outputEncoder = bsp_as5600GetRawAngle();
        HAL_Delay(10);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == (&htim2)) {}
    if(htim == (&htim3)) {}
}

void funcWritePwm(_iq UaPer, _iq UbPer, _iq UcPer)
{
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, _IQtoF(UaPer) * _COUNTER_PERIOD_TIM1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, _IQtoF(UbPer) * _COUNTER_PERIOD_TIM1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, _IQtoF(UcPer) * _COUNTER_PERIOD_TIM1);
}

_iq getRawAngle()
{
    uint16_t rxBuf_SPI;
    uint16_t txBuf_SPI = 0x00;
    uint16_t rawAngle;
    GPIOA->BRR = GPIO_PIN_15;
    spi_transmit_receive(txBuf_SPI, &rxBuf_SPI);
    GPIOA->BSRR = GPIO_PIN_15;
    rawAngle = (rxBuf_SPI & 0xFFFC) >> 2;
    return _IQdiv( _IQ(rawAngle), 85445656);//16384/2PI=2,607.5945876
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
//    timein = micros();
    _iq soa, sob, soc;
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    ADC_Values[0] = hadc->Instance->JDR1;
    ADC_Values[1] = hadc->Instance->JDR2;
    ADC_Values[2] = hadc->Instance->JDR3;

    soa = _IQdiv(_IQ(2048 - ADC_Values[0]), 24397266); //1/1/4095*3300/Gcsa/Rsense/1000=744.5454545f 计算公式见手册41页
    sob = _IQdiv(_IQ(2048 - ADC_Values[1]), 24397266);
    soc = _IQdiv(_IQ(2048 - ADC_Values[2]), 24397266);

    setCurrents(soa, sob, soc);
    setAngle(getRawAngle());

    if(setFlag)
    {
        DFOC_M0_SET_ANGLE_PID(_IQ(ang_p), _IQ(ang_i), _IQ(ang_d), _IQ(65535), _IQ(ang_limit));
        DFOC_M0_SET_CURRENT_PID(_IQ(cur_p), _IQ(cur_i), _IQ(cur_d), _IQ(65535));
        DFOC_M0_SET_VEL_PID(_IQ(vel_p), _IQ(vel_i), _IQ(vel_d), _IQ(65535), _IQ(vel_limit));
    }

    //    DFOC_M0_setTorque(_IQ(cur_target));
    //    DFOC_M0_setVelocity(_IQ(vel_target));
    //    DFOC_M0_set_Force_Angle(_IQ(pos_target));
    DFOC_M0_set_Velocity_Angle(_IQ(pos_target));

//    soaBuf[soaBufFlag] = motor_velocity;
//    soaBufFlag++;
//    if(soaBufFlag > 2047)
//        soaBufFlag = 0;
//    timeout = micros() - timein;
}

void spi_transmit_receive(uint16_t data_in, uint16_t *data_out)
{
    *data_out = 0;
    // Wait until TXE flag is set to send data
    while(!LL_SPI_IsActiveFlag_TXE(SPI1));

    // Transmit data in 16 Bit mode
    LL_SPI_TransmitData16(SPI1, data_in);

    // Check BSY flag
    while(LL_SPI_IsActiveFlag_BSY(SPI1));

    while(!LL_SPI_IsActiveFlag_RXNE(SPI1));

    // Read 16-Bits in the data register
    *data_out = LL_SPI_ReceiveData16(SPI1);
}

uint16_t bsp_as5600GetRawAngle(void)
{
    uint16_t raw_angle;
    uint8_t buffer[2] = {0};
    uint8_t raw_angle_register = 0x0C;

    HAL_I2C_Master_Transmit(&hi2c1, AS5600_ADDR, &raw_angle_register, 1, 0xff);
    HAL_I2C_Master_Receive(&hi2c1, AS5600_ADDR, buffer, 2, 0xff);

    raw_angle = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    return raw_angle;
}


