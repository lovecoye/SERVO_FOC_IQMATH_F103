#include "getmicros.h"
#include "stm32f1xx_hal.h"

//����ƽ̨��ͬ�޸Ļ�ȡϵͳʱ��
__STATIC_INLINE uint32_t GXT_SYSTICK_IsActiveCounterFlag(void)
{
    return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

static uint32_t getCurrentMicros(void)
{
    /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
    GXT_SYSTICK_IsActiveCounterFlag();
    uint32_t m = HAL_GetTick();
    const uint32_t tms = SysTick->LOAD + 1;
    __IO uint32_t u = tms - SysTick->VAL;
    if (GXT_SYSTICK_IsActiveCounterFlag())
    {
        m = HAL_GetTick();
        u = tms - SysTick->VAL;
    }
    return (m * 1000 + (u * 1000) / tms);
}
//����ƽ̨��ͬ�޸Ļ�ȡϵͳʱ��

//��ȡϵͳʱ�䣬��λus
uint32_t micros(void)
{
    return getCurrentMicros();
}

void delay(uint32_t _ms)
{
	HAL_Delay(_ms);
}
