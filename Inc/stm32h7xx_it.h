/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
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
#ifndef __STM32H7xx_IT_H
#define __STM32H7xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hw_config.h"

extern volatile float count;

extern volatile uint8_t Print_Ready;

extern float ia_samples[MAX_SAMPLES];
extern float ib_samples[MAX_SAMPLES];
extern float ic_samples[MAX_SAMPLES];
extern float id_samples[MAX_SAMPLES];
extern float ie_samples[MAX_SAMPLES];
extern float if_samples[MAX_SAMPLES];
extern float t_samples[MAX_SAMPLES];

extern uint32_t real_t_samples[MAX_SAMPLES];

extern uint32_t sample_count;
extern uint8_t collecting_samples;


extern float pi_t_samples[MAX_SAMPLES];
extern uint32_t pi_sample_count;
extern uint8_t pi_collecting_samples;
extern uint8_t PI_Print_Ready;
extern uint8_t pi_control_ready;
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
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void TIM2_IRQHandler(void);
void OTG_FS_IRQHandler(void);
/* USER CODE BEGIN EFP */
float Ramp(void);
uint32_t getTimer5Delta(void);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32H7xx_IT_H */
