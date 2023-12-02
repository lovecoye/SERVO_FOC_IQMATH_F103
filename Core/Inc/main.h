/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

#include "stm32f1xx_ll_spi.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_dma.h"

#include "stm32f1xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SOA_Pin GPIO_PIN_3
#define SOA_GPIO_Port GPIOA
#define SOB_Pin GPIO_PIN_4
#define SOB_GPIO_Port GPIOA
#define SOC_Pin GPIO_PIN_5
#define SOC_GPIO_Port GPIOA
#define AI3V3_Pin GPIO_PIN_6
#define AI3V3_GPIO_Port GPIOA
#define AI24V_Pin GPIO_PIN_7
#define AI24V_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOB
#define nFAULT_Pin GPIO_PIN_1
#define nFAULT_GPIO_Port GPIOB
#define AS5600_DIR_Pin GPIO_PIN_2
#define AS5600_DIR_GPIO_Port GPIOB
#define MA730_CS_Pin GPIO_PIN_15
#define MA730_CS_GPIO_Port GPIOA
#define MA730_SCK_Pin GPIO_PIN_3
#define MA730_SCK_GPIO_Port GPIOB
#define MA730_MISO_Pin GPIO_PIN_4
#define MA730_MISO_GPIO_Port GPIOB
#define MA730_MOSI_Pin GPIO_PIN_5
#define MA730_MOSI_GPIO_Port GPIOB
#define ENABLE_Pin GPIO_PIN_6
#define ENABLE_GPIO_Port GPIOB
#define CAL_Pin GPIO_PIN_7
#define CAL_GPIO_Port GPIOB
#define AS5600_SCL_Pin GPIO_PIN_8
#define AS5600_SCL_GPIO_Port GPIOB
#define AS5600_SDA_Pin GPIO_PIN_9
#define AS5600_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
