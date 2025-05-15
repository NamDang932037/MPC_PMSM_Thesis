#ifndef HW_CONFIG_H
#define HW_CONFIG_H

/* Testing  define*/
//#define PI_TEST
//#define MPC_TEST
#define MAX_SAMPLES 30000

/* Timer and PWM */
#define TIM_PWM			htim2							// PWM/ISR timer handle
#define TIM_CH_U		TIM_CHANNEL_1					// Terminal U timer channel
#define TIM_CH_V		TIM_CHANNEL_2					// Terminal V timer channel
#define TIM_CH_W		TIM_CHANNEL_3					// Terminal W timer channel
#define INVERT_DTC		0								// PWM inverting (1) or non-inverting (0)

/* ISRs */
#define PWM_ISR			TIM2_IRQn						// PWM Timer ISR

/* ADC */
#define ADC_CH_MAIN		hadc1							// ADC channel handle which drives simultaneous mode
#define ADC_CH_IA		hadc1							// Phase A current sense ADC channel handle.  0 = unused
#define ADC_CH_IB		hadc2							// Phase B current sense ADC channel handle.  0 = unused
#define ADC_CH_IC		hadc3							// Phase C current sense ADC channel handle.  0 = unused
#define ADC_CH_VBUS		hadc3							// Bus voltage ADC channel handle.  0 = unused

/* DRV Gate drive */
#define ENABLE_PIN 		GPIOC, GPIO_PIN_14  			// Enable gate drive pin.
#define DRV_SPI			hspi3							// DRV SPI handle
#define DRV_CS			GPIOB, GPIO_PIN_0				// DRV CS pin
#define DRV_HIZ			GPIOC, GPIO_PIN_15				// DRV HI-Z Pin
/* SPI encoder */
#define ENC_SPI			hspi4							// Encoder SPI handle
#define ENC_CS			GPIOE, GPIO_PIN_3				// Encoder SPI CS pin
#define ENC_CPR			16384							// Encoder counts per revolution
#define INV_CPR			1.0f/ENC_CPR
#define ENC_READ_WORD	0x8021							// Encoder read command
#define ENC_READ_VEL	0xC031

/* Misc. GPIO */
#define LED         	GPIOA, GPIO_PIN_3				// LED Pin
#define TIM_MULT		1e-8f

/* Other hardware-related constants */
#define I_SCALE 			0.004577636719f    			// Amps per A/D Count at 20X amplifier gain (Vref/ADC_Count*R_Shunt*CSA_GAIN)
#define V_SCALE 			0.001571679255f       		// Bus volts per A/D Count: (Vref/2^n-1)*(103/3) (Check your hardware)
#define DTC_MAX 			0.9344f          				// Max duty cycle
#define DTC_MIN 			0.0656f          				// Min duty cycle
//#define DTC_MAX 			1.0f          				// Max duty cycle
//#define DTC_MIN 			0.0f          				// Min duty cycle
#define BLEND_MIN			0.2f
#define BLEND_MAX			0.6f
#define BLEND_REGION		0.4f
#define COMP_MAG			0.05
#define COMP_OFF			0.027
#define DTC_COMP 			100e-9f          			// deadtime compensation (100 ns / 25 us)
#define DT_ENC				2.5e-5f						// Loop period
#define DT					5.0e-5f						// <Testing 400Hz>
#define EN_ENC_LINEARIZATION 1							// Enable/disable encoder linearization
#define V_BUS_MAX			30.0f						// max drive voltage (faults above this)

/* Current controller */
#define L_D 				2.4521034641171843e-05f		// D axis inductance
#define L_Q 				2.4521034641171843e-05f		// Q axis inductance

/* PI Current */
//KI = w3dB*R;		    //KP = w3dB*L;	w=2*pi*f; f = 100Hz
#define K_D 				0.1541f                	// Loop gain,  Volts/Amp
#define K_Q 				0.1541f                	// Loop gain,  Volts/Amp
//#define K_D 				0.1741f                	// Loop gain,  Volts/Amp
//#define K_Q 				0.1741f                	// Loop gain,  Volts/Amp
#define KI_D 				320.41997955f             // PI zero, in radians per sample
#define KI_Q 				320.41997955f             // PI zero, in radians per sample


#define MAX_VOLTAGE_RATIO	0.92
//Mechanical model
//Nice
//#define Bm					3.149896550641861e-6f
//#define J					6.98637931297184e-6f

//Testing
#define Bm					3.159896550641861e-6f
#define J					7.028499038470909e-6f

//#define Bm					4.107865515e-6f
//#define J					9.13704875e-6f
#define Te_max				0.30f						// Max eletromatic torque
//MPC Parameter
#define NP					5
#define NC					3
#define DT_MPC				0.0002f
#define K_SCALE 			0.00012f             		// K_loop/Loop BW (Hz) 0.0042
#define DT_KF				0.0002f
#define OVERMODULATION 		1.1547f        				// 1.0 = no overmodulation
//#define OVERMODULATION 		1.0f        				// 1.0 = no overmodulation


#define COGGING_TABLE_SIZE  1024
#define COGGING_SCALE		1

//Low pass filter for current sense and Vbus sense
#define CURRENT_FILT_ALPHA	.01f						// 1st order d/q current filter (not used in control)
#define VBUS_FILT_ALPHA		.01f						// 1st order bus voltage filter

#define D_INT_LIM V_BUS/(K_D*KI_D)  					// Amps*samples
#define Q_INT_LIM V_BUS/(K_Q*KI_Q)  					// Amps*samples
#define TORQUE_CONSTANT		0.02809053911f
// PLL filter constants
#define PLL_KP                  (2.0f * 2.0f * M_PI * 100.0f)  //100Hz bandwidth
#define PLL_KI                  (2.0f * M_PI * 100.0f * 2.0f * M_PI * 100.0f)
#define MAX_VELOCITY_SCALE       0.125f  				// 1/8 revolution per sample
#define VELOCITY_FILTER_ALPHA    0.01f   				// Exponential filter constant
#define POSITION_FILTER_HZ       50.0f   				// 50Hz cutoff frequency

#define CPR                      16384.0f  				// Counts per revolution (15-bit encoder)
#define RAD_PER_CNT             (TWO_PI_F / CPR)  		// rad/count
#define VELOCITY_FILTER_HZ       100.0f     				// 100Hz cutoff for PLL
#define MAX_VELOCITY_RAD         100.0f    				// Maximum velocity in rad/s


#endif
