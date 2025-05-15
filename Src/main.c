/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
#include "usbd_def.h"
#include <stdbool.h>
#include "hw_config.h"
#include "position_sensor.h"

#include "drv8323.h"
#include "foc.h"
#include "calibration.h"
#include "preference_writer.h"
#include "user_config.h"
#include "fsm.h"
#include "math_ops.h"
#include "calibration.h"
#include "structs.h"
#include "flash_writer.h"
#include "KF.h"
#include "MPC_Trajectory.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void write_config_values(void);
void read_and_print_config(void);
uint32_t getTimer17Delta(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define VERSION_NUM 666.0f
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
EncoderStruct comm_encoder;
DRVStruct drv = {0};
CalStruct comm_encoder_cal;
ControllerStruct controller;
PreferenceWriter prefs;

ObserverStruct observer;
COMStruct com;
FSMStruct state;


float __float_reg[64];
int __int_reg[256];


int *error_array = NULL;
int *lut_array = NULL;


uint8_t buffer[1];
uint16_t value = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char *privatemessege = "Hello, Today we booted into this board\n";
volatile float count = 0;
volatile uint32_t count_tim17 = 0;
volatile uint32_t delta = 0;
volatile uint32_t delta_1us = 0;
volatile uint32_t Check = 0;
volatile uint8_t Print_Ready = 0;


float ia_samples[MAX_SAMPLES];
float ib_samples[MAX_SAMPLES];
float ic_samples[MAX_SAMPLES];
float id_samples[MAX_SAMPLES];
float ie_samples[MAX_SAMPLES];
float if_samples[MAX_SAMPLES];
float t_samples[MAX_SAMPLES];
uint32_t real_t_samples[MAX_SAMPLES];

uint32_t sample_count = 0;
uint8_t collecting_samples = 0;

volatile uint32_t Time_Capture[10];

float pi_t_samples[MAX_SAMPLES];
uint32_t pi_sample_count = 0;
uint8_t pi_collecting_samples = 0;
uint8_t PI_Print_Ready = 0;
uint8_t pi_control_ready = 0;


extern USBD_HandleTypeDef hUsbDeviceFS;

int _write(int file, char *ptr, int len) {

     if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
         return len;
     }

     while (CDC_Transmit_FS((uint8_t*)ptr, len) == USBD_BUSY) {
     }
     return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
//  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
//  timeout = 0xFFFF;
//  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
//  if ( timeout < 0 )
//  {
//  Error_Handler();
//  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
//timeout = 0xFFFF;
//while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
//if ( timeout < 0 )
//{
//Error_Handler();
//}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FMC_Init();
  MX_TIM17_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM5_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */


  HAL_Delay(5000);

  if (HAL_ADCEx_Calibration_Start(&ADC_CH_IA, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
		Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&ADC_CH_IB, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
		Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&ADC_CH_VBUS, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
		Error_Handler();
  }

  printf("\r\nFirmware Version Number: %.2f\r\n", VERSION_NUM);

    preference_writer_init(&prefs, 6);
    preference_writer_load(prefs);

    /* Sanitize configs in case flash is empty*/
    if(E_ZERO==-1){E_ZERO = 0;}
    if(M_ZERO==-1){M_ZERO = 0;}
    if(isnan(I_BW) || I_BW==-1){I_BW = 100;}
    if(isnan(I_MAX) || I_MAX ==-1){I_MAX=20;}
    if(isnan(I_FW_MAX) || I_FW_MAX ==-1){I_FW_MAX=20;}
    if(isnan(R_NOMINAL) || R_NOMINAL==-1){R_NOMINAL = 0.0f;}
    if(isnan(TEMP_MAX) || TEMP_MAX==-1){TEMP_MAX = 100.0f;}
    if(isnan(I_MAX_CONT) || I_MAX_CONT==-1){I_MAX_CONT = 14.0f;}
    if(isnan(I_CAL)||I_CAL==-1){I_CAL = 5.0f;}
    if(isnan(PPAIRS) || PPAIRS==-1){PPAIRS = 7.0f;}
    if(isnan(GR) || GR==-1){GR = 1.0f;}
    if(isnan(KT) || KT==-1){KT = 0.02809053911f;}
    if(isnan(KP_MAX) || KP_MAX==-1){KP_MAX = 500.0f;}
    if(isnan(KD_MAX) || KD_MAX==-1){KD_MAX = 5.0f;}
    if(isnan(P_MAX)){P_MAX = 50.0f;}
    if(isnan(P_MIN)){P_MIN = -50.0f;}
    if(isnan(V_MAX)){V_MAX = 100.0f;}
    if(isnan(V_MIN)){V_MIN = 100.0f;}

    /* Controller Setup */
    controller.phase_order = 0;
    PHASE_ORDER = 0;
    KT = 0.02809053911f;
    if(PHASE_ORDER){							// Timer channel to phase mapping

    }
    else {

    }

    init_controller_params(&controller);

    /* Flash Programming*/
    //  write_config_values();
    //  HAL_Delay(1000);
    //  read_and_print_config();

    /* Encoder setup */
    memset(&comm_encoder_cal.cal_position, 0, sizeof(EncoderStruct));
    comm_encoder.m_zero = 0;
    comm_encoder.e_zero = 0;
    comm_encoder.ppairs = 7;

    ps_checkmode();
    ps_validcheck();

    if(EN_ENC_LINEARIZATION) {
  	  // Copy the linearization lookup table
  	  memcpy(&comm_encoder.offset_lut, &ENCODER_LUT, sizeof(comm_encoder.offset_lut));
    }
    else{
  	  memset(&comm_encoder.offset_lut, 0, sizeof(comm_encoder.offset_lut));
    }
    // clear the noisy data when the encoder first turns on
    ps_warmup(&comm_encoder, 100);

    zero_current(&controller);


    /* DRV8353S config */


    HAL_GPIO_WritePin(DRV_CS, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(ENABLE_PIN, GPIO_PIN_SET);
    HAL_Delay(100);

    drv_print_faults_values(drv);
    HAL_Delay(10);

    /* Calib if need */
    //	drv_calibrate(drv);
    //	HAL_Delay(1);

  	printf("Let me check first\r\n");
  	value = drv_read_register(drv, DCR);
  	printf("DCR_VAL = 0x%04X\r\n", value);
  	HAL_Delay(10);
  	value = drv_read_register(drv, HSR);
  	printf("HSR_VAL = 0x%04X\r\n", value);
  	HAL_Delay(10);
  	value = drv_read_register(drv, LSR);
  	printf("LSR_VAL = 0x%04X\r\n", value);
  	HAL_Delay(10);
  	value = drv_read_register(drv, OCPCR);
  	printf("OCPCR_VAL = 0x%04X\r\n", value);
  	HAL_Delay(10);
  	value = drv_read_register(drv, CSACR);
  	printf("CSACR_VAL = 0x%04X\r\n", value);
  	HAL_Delay(10);
  	printf("Check done - Continue to config\r\n");

  	//Drive control register
  	drv_write_DCR(drv, 0x0, 0x0, DIS_GDF_EN, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x0);
  	HAL_Delay(50);
  	value = drv_read_register(drv, DCR);
  	printf("DCR_VAL AFTER = 0x%04X\r\n", value);
  	HAL_Delay(10);

  	//Gate Drive HS
  	drv_write_HSR(drv, LOCK_OFF, IDRIVEP_HS_150MA, IDRIVEN_HS_300MA);
  	HAL_Delay(10);
  	value = drv_read_register(drv, HSR);
  	printf("HSR_VAL AFTER = 0x%04X\r\n", value);
  	HAL_Delay(10);

  	//Gate Drive LS
  	drv_write_LSR(drv, 0x01, TDRIVE_1000NS, IDRIVEP_LS_150MA, IDRIVEN_LS_300MA);
  	HAL_Delay(10);
  	value = drv_read_register(drv, LSR);
  	printf("LSR_VAL AFTER = 0x%04X\r\n", value);
  	HAL_Delay(10);

  	//Over Current Protection Control Register
  	drv_write_OCPCR(drv, TRETRY_8MS, DEADTIME_100NS, OCP_LATCH, OCP_DEG_4US, VDS_LVL_0_7);
  	HAL_Delay(10);
  	value = drv_read_register(drv, OCPCR);
  	printf("OCPCR_VAL AFTER = 0x%04X\r\n", value);
  	HAL_Delay(10);

  	//Current Sense Amplifier Control Register
  	drv_write_CSACR(drv, 0x0, VREF_DIV_2, 0x0, CSA_GAIN_20, 0x0, 0x0, 0x0, 0x0, SEN_LVL_0_5);
  	HAL_Delay(10);
  	value = drv_read_register(drv, CSACR);
  	printf("CSACR_VAL AFTER = 0x%04X\r\n", value);
  	HAL_Delay(10);



  	drv_write_CAL_MODE(drv, 0x01);
  	HAL_Delay(10);
  	value = drv_read_register(drv, CAL);
  	printf("CAL_VAL = 0x%04X\r\n", value);
  	HAL_Delay(10);

  	drv_print_faults_values(drv);
  	HAL_Delay(10);

  	printf("Check again\r\n");
  	value = drv_read_register(drv, DCR);
  	printf("DCR_VAL = 0x%04X\r\n", value);
  	HAL_Delay(10);
  	value = drv_read_register(drv, HSR);
  	printf("HSR_VAL = 0x%04X\r\n", value);
  	HAL_Delay(10);
  	value = drv_read_register(drv, LSR);
  	printf("LSR_VAL = 0x%04X\r\n", value);
  	HAL_Delay(10);
  	value = drv_read_register(drv, OCPCR);
  	printf("OCPCR_VAL = 0x%04X\r\n", value);
  	HAL_Delay(10);
  	value = drv_read_register(drv, CSACR);
  	printf("CSACR_VAL = 0x%04X\r\n", value);
  	HAL_Delay(10);

  	offset_current(&controller);

  	printf("ADC A OFFSET: %d     ADC B OFFSET: %d		ADC C OFFSET: %d\r\n",
  			controller.adc_a_offset, controller.adc_b_offset, controller.adc_c_offset);

    HAL_TIM_PWM_Start(&TIM_PWM, TIM_CH_U);
    HAL_TIM_PWM_Start(&TIM_PWM, TIM_CH_V);
    HAL_TIM_PWM_Start(&TIM_PWM, TIM_CH_W);

  	/* Start the FSM */
  	state.state = MENU_MODE;
  	state.next_state = MENU_MODE;
  	state.ready = 1;
  	enter_menu_state();
  	controller.loop_count = 0;
    controller.kf_done = 1;
    controller.kf_firsttime = 1;
    HAL_NVIC_EnableIRQ(PWM_ISR);
    HAL_TIM_Base_Start_IT(&TIM_PWM);
    HAL_TIM_Base_Start(&htim5);
    HAL_TIM_Base_Start(&htim17);



  /* USER CODE END 2 */


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// Print Iabc when Ud = 0; Uq = constant
	if(Print_Ready) {
	  for(uint32_t i = 0; i < sample_count; i++) {
//		  printf("%.4f          %.4f          %.4f          %.4f          %.4f          %.4f          %.6f\r\n",
//				   ia_samples[i], ib_samples[i], ic_samples[i], id_samples[i], ie_samples[i], if_samples[i], t_samples[i]);
//		  printf("%.4f          %.4f          %.4f          %.4f          %lu\r\n",
//				   ia_samples[i], ib_samples[i], ic_samples[i], id_samples[i], real_t_samples[i]);
		  printf("%.4f          %.4f          %.6f\r\n",
				   ie_samples[i], if_samples[i], t_samples[i]);
			// Delay nhỏ để tránh tràn buffer USB
			HAL_Delay(1);
	  }

	  // Reset các biến
	  Print_Ready = 0;
	  sample_count = 0;
	}


//	  if(PI_Print_Ready) {
//		  for(uint32_t i = 0; i < pi_sample_count; i++) {
//			  printf("%.4f    %.4f    %.4f\r\n",
//			  id_samples[i], iq_samples[i], pi_t_samples[i]);
//			  HAL_Delay(1);
//		  }
//
//		  // Reset các biến
//		  PI_Print_Ready = 0;
//		  pi_sample_count = 0;
//		  pi_collecting_samples = 0;
//		  pi_control_ready = 0;
//		  controller.k_d = 0.0f;
//		  controller.ki_d = 0.0f;
//		  controller.k_q = 0.0f;
//		  controller.ki_q = 0.0f;
//		  controller.i_q_des = 0.0f;
//
//		  // Đảm bảo tắt điều khiển
//		  zero_current(&controller);
//	  }


	  //Check Fault Pin
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_3));

	if(controller.kf_flag == 0 && controller.enable_kf && controller.kf_done) {
//		TIM17->CNT = 0;
		delta_1us = getTimer17Delta();
		KF(&controller, &comm_encoder);

		MPC_Calculate(&controller);
		controller.kf_done = 0;
//		delta_1us = TIM17->CNT;

	}


	if(state.Received) {
		char c = buffer[0];
		update_fsm(&state, c);
		state.Received = 0;
	}
	  // Main switch case
		 /* state transition management */
	 if(state.next_state != state.state){
		 fsm_exit_state(&state);				// safely exit the old state
		 if(state.ready){						// if the previous state is ready, enter the new state
			 state.state = state.next_state;
			 fsm_enter_state(&state);
		 }
	 }

	  switch(state.state) {
		case ENCODER_MODE:
			ps_print(&comm_encoder, DT);
			HAL_Delay(200);
			break;

		case MOTOR_MODE:
			// TODO: Check Input (Pdes, vdes)
			if(controller.p_des == 0 && controller.v_des == 0) {
				// Controller not init, exit
				break;
			}
			else {
				// Controller is ready, perpare to go
				if(controller.Control_Ready == 0 && controller.Valid_Control_Cmd) {
					printf("Motor will be controlled in 5 secs\r\n");
					for(int i = 0; i < 10; i++) {
						HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
						HAL_Delay(300);
					}
					controller.Control_Ready = 1;
					controller.t = 0;

				}
			}

			break;

		case MENU_MODE:

			break;
		case SETUP_MODE:

			break;
		case CALIBRATION_MODE:
			break;


	  }



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 16;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_QSPI|RCC_PERIPHCLK_ADC
                              |RCC_PERIPHCLK_SPI4;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 32;
  PeriphClkInitStruct.PLL2.PLL2P = 6;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 4;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void write_config_values(void) {
    // Mở flash trước
    preference_writer_open(&prefs);

    // Ghi các giá trị integer
    preference_writer_write_int(E_ZERO, 0);
    preference_writer_write_int(M_ZERO, 1);
//    preference_writer_write_int(CAN_ID, 2);
//    preference_writer_write_int(CAN_MASTER, 3);
//    preference_writer_write_int(CAN_TIMEOUT, 4);

    // Ghi các giá trị float
    preference_writer_write_float(I_BW, 2);
    preference_writer_write_float(I_MAX, 3);
    preference_writer_write_float(I_FW_MAX, 6);
    preference_writer_write_float(R_NOMINAL, 7);
    preference_writer_write_float(TEMP_MAX, 8);
    preference_writer_write_float(I_MAX_CONT, 9);
    preference_writer_write_float(PPAIRS, 10);
    preference_writer_write_float(KT, 14);
    preference_writer_write_float(GR, 17);
    preference_writer_write_float(I_CAL, 18);
    preference_writer_write_float(P_MIN, 19);
    preference_writer_write_float(P_MAX, 20);
    preference_writer_write_float(V_MIN, 21);
    preference_writer_write_float(V_MAX, 22);

    // Ghi buffer vào flash
    preference_writer_flush(&prefs);
    // Đóng flash
    preference_writer_close(&prefs);

    // Thêm delay để đảm bảo hoàn tất
    HAL_Delay(100);
}


void read_and_print_config(void) {
    preference_writer_load(prefs);

    printf("Init Values:\r\n");
    printf("E_ZERO: %d\r\n", E_ZERO);
    printf("M_ZERO: %d\r\n", M_ZERO);
    printf("I_BW: %.2f\r\n", I_BW);
    printf("I_MAX: %.2f\r\n", I_MAX);
    printf("I_FW_MAX: %.2f\r\n", I_FW_MAX);
//    printf("CAN_ID: %d\r\n", CAN_ID);
//    printf("CAN_MASTER: %d\r\n", CAN_MASTER);
//    printf("CAN_TIMEOUT: %d\r\n", CAN_TIMEOUT);
    printf("R_NOMINAL: %.2f\r\n", R_NOMINAL);
    printf("TEMP_MAX: %.2f\r\n", TEMP_MAX);
    printf("I_MAX_CONT: %.2f\r\n", I_MAX_CONT);
    printf("I_CAL: %.2f\r\n", I_CAL);
    printf("PPAIRS: %.2f\r\n", PPAIRS);
    printf("GR: %.2f\r\n", GR);
    printf("KT: %.2f\r\n", KT);
    printf("KP_MAX: %.2f\r\n", KP_MAX);
    printf("KD_MAX: %.2f\r\n", KD_MAX);
    printf("P_MAX: %.2f\r\n", P_MAX);
    printf("P_MIN: %.2f\r\n", P_MIN);
    printf("V_MAX: %.2f\r\n", V_MAX);
    printf("V_MIN: %.2f\r\n", V_MIN);
}

uint32_t getTimer17Delta(void) {
    uint32_t current_count = TIM17->CNT;
    uint32_t delta;

    if (current_count >= count_tim17) {
        delta = current_count - count_tim17;
    } else {
        // Trường hợp bộ đếm bị tràn
        delta = (0xFFFF - count_tim17) + current_count + 1;
    }

    count_tim17 = current_count;  // Cập nhật giá trị count
    return delta;
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x60000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x20020000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
