/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define MTR_HI_Z_Pin GPIO_PIN_15
#define MTR_HI_Z_GPIO_Port GPIOC
#define MTR_ENA_Pin GPIO_PIN_14
#define MTR_ENA_GPIO_Port GPIOC
#define TLE_CS_Pin GPIO_PIN_3
#define TLE_CS_GPIO_Port GPIOE
#define ETC_IRQ_Pin GPIO_PIN_2
#define ETC_IRQ_GPIO_Port GPIOF
#define ETC_IRQ_EXTI_IRQn EXTI2_IRQn
#define ETC_SYNC1_Pin GPIO_PIN_1
#define ETC_SYNC1_GPIO_Port GPIOF
#define ETC_SYNC1_EXTI_IRQn EXTI1_IRQn
#define ETC_SYNC0_Pin GPIO_PIN_0
#define ETC_SYNC0_GPIO_Port GPIOF
#define ETC_SYNC0_EXTI_IRQn EXTI0_IRQn
#define MTR_FAULT_Pin GPIO_PIN_3
#define MTR_FAULT_GPIO_Port GPIOF
#define ETC_RST_Pin GPIO_PIN_3
#define ETC_RST_GPIO_Port GPIOG
#define ADC_SOC_Pin GPIO_PIN_2
#define ADC_SOC_GPIO_Port GPIOC
#define ADC_SOB_Pin GPIO_PIN_6
#define ADC_SOB_GPIO_Port GPIOA
#define ADC_SOA_Pin GPIO_PIN_4
#define ADC_SOA_GPIO_Port GPIOC
#define MTR_CS_Pin GPIO_PIN_0
#define MTR_CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
