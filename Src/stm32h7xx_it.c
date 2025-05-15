/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
#include <stdio.h>
#include "structs.h"
#include <string.h>

#include "spi.h"
#include "gpio.h"
#include "adc.h"
#include "foc.h"


#include "position_sensor.h"
#include "hw_config.h"
#include "user_config.h"
#include "structs.h"
#include "fsm.h"
#include "KF.h"
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint8_t buffer[1];
extern volatile uint32_t delta;
extern volatile uint32_t delta_1us;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern TIM_HandleTypeDef htim2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
	printf("Write to internal Flash failed - Kill for safety");
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
	 HAL_Delay(100);
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ETC_SYNC0_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ETC_SYNC1_Pin);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ETC_IRQ_Pin);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
		if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET) {
			__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
			// Đọc bit DIR để xem đang đếm lên hay xuống
			if ((TIM2->CR1 & TIM_CR1_DIR) == 0) {
				// Disable Controller ISR
				NVIC_DisableIRQ(PWM_ISR);
				__HAL_TIM_CLEAR_IT(&TIM_PWM, TIM_IT_UPDATE);

				delta = getTimer5Delta();
				//Disable when testing MPC

				if(controller.Control_Ready) {
					controller.t += (float)delta*TIM_MULT;
				}

				// Currents and Bus sensing
//				TIM5->CNT = 0;
				analog_sample(&controller);

				ps_sample(&comm_encoder, (float)delta*TIM_MULT);

				 //Disable when testing MPC
			    controller.theta_elec = comm_encoder.elec_angle;
			    controller.dtheta_elec = comm_encoder.elec_velocity;
			    controller.dtheta_mech = comm_encoder.filtered_vel;
			    controller.theta_mech = comm_encoder.angle_multiturn[0];

//				delta = TIM5->CNT; //1 tick = 10ns

				dq0(comm_encoder.elec_angle, controller.i_a, controller.i_b, controller.i_c, &controller.i_d, &controller.i_q);

				 //Disable when testing MPC
				controller.Pre_Te = controller.Te;
			    controller.Te = controller.i_q*TORQUE_CONSTANT;

			    if(delta < 2600) {
			    	controller.kf_flag++;
			    }
			    else if(delta < 5200) {
			    	controller.kf_flag += 2;
			    }
				if(controller.kf_flag >= 8) {
					controller.kf_flag = 0;
					controller.kf_done = 1;
					  //Enable when testing MPC
//					  controller.enable_kf = 1;
				}


				if(controller.t >= 5 && controller.t <= 7.5f) {
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
//					  collecting_samples = 1;

					  // Disable when testing MPC
					  controller.enable_kf = 1;

					  if(fabs(controller.i_a) >= 25 || fabs(controller.i_b) >= 25 || fabs(controller.i_c) >= 25) {
						  controller.fault = 1;
						  controller.enable_kf = 0;
							controller.p_des = 0;
							controller.v_des = 0;
							controller.mode = 0;
							controller.Valid_Control_Cmd = 0;
							controller.Control_Ready = 0;
						  zero_current(&controller);
					  }
					  if(fabs(comm_encoder.filtered_vel) >= 70) {
						  controller.fault = 1;
						  controller.enable_kf = 0;
							controller.p_des = 0;
							controller.v_des = 0;
							controller.mode = 0;
							controller.Valid_Control_Cmd = 0;
							controller.Control_Ready = 0;
						  zero_current(&controller);
					  }
					  if(!controller.fault) {


						  // PI_Currents Tunning
//						  controller.i_d_des = 0.0f;
//						  controller.i_q_des = 1.5f;
//						  controller.k_d = K_D*10.0f;
//						  controller.k_q = K_D*10.0f;
//
//						  controller.ki_d = KI_D*10.0f;
//						  controller.ki_q = KI_D*10.0f;
						  FOC(&controller, &comm_encoder);

						  //Testing Open-Loop
//						  controller.v_d = 0.0f;
//						  controller.v_q = 0.0f;
//						  SVPWM_v2(&controller, &comm_encoder);


						  // Encoder Calibration - Ramp
//						  controller.v_d = 0.9;
//						  controller.v_q = 0;
//						  comm_encoder.mecha_pos_ramp = Ramp();
//						  comm_encoder.elec_pos_ramp = wrap_zero_to_two_pi(comm_encoder.mecha_pos_ramp*comm_encoder.ppairs);
//						  SVPWM_v2(&controller, &comm_encoder);
					  }
				}

				else if(controller.t >= 7.5f && collecting_samples) {
					controller.p_des = 0;
					controller.v_des = 0;
					controller.mode = 0;
					controller.Valid_Control_Cmd = 0;
					controller.Control_Ready = 0;
					controller.kf_firsttime = 1;
					controller.Te_des = 0;
					controller.i_q_des = 0;
					controller.d_int = 0;
					controller.q_int = 0;

					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
					controller.enable_kf = 0;
					collecting_samples = 0;
					Print_Ready = 1;
					zero_current(&controller);
				}
				else {
					zero_current(&controller);
				}

				if(controller.t >= 5.00f && controller.t <= 7.5f) {
					collecting_samples = 1;
					  if(controller.loop_count % 2 == 0) {
						  if(sample_count < MAX_SAMPLES) {

//							  ia_samples[sample_count] = controller.p_rs;
//							  ib_samples[sample_count] = controller.p_plan;
//							  ic_samples[sample_count] = controller.v_rs;
//							  id_samples[sample_count] = controller.v_plan;
//							  ie_samples[sample_count] = controller.Tm_Filtered;

//							  ia_samples[sample_count] = comm_encoder.angle_singleturn;

//							  ia_samples[sample_count] = controller.theta_mech;
//							  ib_samples[sample_count] = controller.p_plan;
//							  ic_samples[sample_count] = controller.dtheta_mech;
//							  id_samples[sample_count] = controller.v_plan;

//							  ic_samples[sample_count] = controller.Te;
//							  id_samples[sample_count] = controller.Te_des;

//							  ia_samples[sample_count] = controller.theta_mech;
//							  ib_samples[sample_count] = controller.pos_estimate;
//							  ic_samples[sample_count] = controller.dtheta_mech;
//							  id_samples[sample_count] = controller.vel_estimate;
//							  ie_samples[sample_count] = controller.Tm_Filtered;
//							  if_samples[sample_count] = controller.Te;

							  ie_samples[sample_count] = controller.Te;
							  if_samples[sample_count] = controller.Te_des;

//								  ia_samples[sample_count] = controller.i_a;
//								  ib_samples[sample_count] = controller.i_b;
//								  ic_samples[sample_count] = controller.i_c;
//							  	  id_samples[sample_count] = controller.v_bus_filt;

//							  ia_samples[sample_count] = controller.theta_mech;
//							  ib_samples[sample_count] = controller.dtheta_mech;
//							  ic_samples[sample_count] = controller.i_q_des;
//							  id_samples[sample_count] = controller.i_q;

//							  ia_samples[sample_count] = controller.dtc_u;
//							  ib_samples[sample_count] = controller.dtc_v;
//							  ic_samples[sample_count] = controller.dtc_w;
//							  id_samples[sample_count] = controller.i_q;

//								  ia_samples[sample_count] = comm_encoder.angle_singleturn;
//								  ib_samples[sample_count] = comm_encoder.elec_angle;
//								  ic_samples[sample_count] = comm_encoder.mecha_pos_ramp;
//								  id_samples[sample_count] = comm_encoder.elec_pos_ramp;

//							  ia_samples[sample_count] = controller.theta_mech;
//							  ib_samples[sample_count] = controller.dtheta_mech;
//							  ic_samples[sample_count] = controller.i_d;
//							  id_samples[sample_count] = controller.i_q;

							  t_samples[sample_count] = controller.t - 5;
//							  real_t_samples[sample_count] = delta_1us;

						  }
						  //Smaple
						  sample_count++;
					  }
				}
			controller.loop_count++;
//			count = TIM5->CNT;		//1 tick = 1ns
			HAL_NVIC_EnableIRQ(PWM_ISR);
		}
	}
}
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */


  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */

float Ramp(void) {
    // Tốc độ góc: 10 RPM = π/3 rad/s
    const float ANGULAR_FREQ = 3*M_PI / 3.0f; // Tốc độ góc (rad/s)

    // Tính giá trị phase dùng controller.t
    float phase = ANGULAR_FREQ * (controller.t - 5);

    // Giữ phase trong khoảng [0, 2π) bằng cách lấy phần dư
    phase = fmodf(phase, 2.0f * M_PI);

    // Đảm bảo phase không âm
    if (phase < 0.0f) {
        phase += 2.0f * M_PI;
    }

    return phase;
}

uint32_t getTimer5Delta(void) {
    uint32_t current_count = TIM5->CNT;
    uint32_t delta;

    if (current_count >= count) {
        delta = current_count - count;
    } else {
        // Trường hợp bộ đếm bị tràn
        delta = (0xFFFFFFFF - count) + current_count + 1;
    }

    count = current_count;  // Cập nhật giá trị count
    return delta;
}
/* USER CODE END 1 */
