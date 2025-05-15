//Update for AS5047
//AS5047 = 10Mhz CPOL = 0; CPHA = 1; SPI Mode 1
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"
#include "structs.h"


void ps_warmup(EncoderStruct * encoder, int n) {
    for(int i = 0; i<n; i++) {
        encoder->spi_tx_word = ENC_READ_WORD;
        HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET);
        HAL_SPI_Transmit(&ENC_SPI, (uint8_t*)encoder->spi_tx_buff, 1, 10);
        HAL_SPI_Receive(&ENC_SPI, (uint8_t*)encoder->spi_rx_buff, 1, 10);
        HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET);
    }
}

// Orginal ver - Backup
void ps_sample(EncoderStruct * encoder, float dt) {
/* Shift around previous samples */
    encoder->old_angle = encoder->angle_singleturn;
    for(int i = N_POS_SAMPLES-1; i>0; i--) {
       encoder->angle_multiturn[i] = encoder->angle_multiturn[i-1];
    }

   /* SPI read/write with minimal delay */
   encoder->spi_tx_word = ENC_READ_WORD;
   HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET);
   HAL_SPI_Transmit(&ENC_SPI, (uint8_t*)encoder->spi_tx_buff, 1, 10);
   HAL_SPI_Receive(&ENC_SPI, (uint8_t*)encoder->spi_rx_buff, 1, 10);
   HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET);
   int16_t raw_signed = (int16_t)(encoder->spi_rx_word & 0x7FFF); // Lấy 15 bit


   if(raw_signed & 0x4000) { // Kiểm tra bit dấu (bit 14)
       raw_signed = -(~raw_signed & 0x3FFF) - 1; // Chuyển về số âm nếu bit dấu = 1
   }
   encoder->raw = raw_signed;

//   if(EN_ENC_LINEARIZATION) {
//       int off_1 = encoder->offset_lut[(abs(encoder->raw))>>9];
//       int off_2 = encoder->offset_lut[((abs(encoder->raw)>>9)+1)%128];
//       int off_interp = off_1 + ((off_2 - off_1)*(abs(encoder->raw) - ((abs(encoder->raw)>>9)<<9))>>9);
//       encoder->count = encoder->raw + (encoder->raw < 0 ? -off_interp : off_interp);
//   } else {
//       encoder->count = encoder->raw;
//   }

    encoder->count = encoder->raw;

    encoder->angle_singleturn = (PI_F * (float)encoder->count) / 16384.0f; 
    /* [0, 2π] */
    encoder->angle_singleturn = wrap_zero_to_two_pi(encoder->angle_singleturn + 0.1039);
    //   encoder->angle_singleturn = wrap_zero_to_two_pi(encoder->angle_singleturn);
    // Rollover for multi-turn
    float angle_diff = encoder->angle_singleturn - encoder->old_angle;
    if(angle_diff > M_PI) {
        encoder->turns--;
    } else if(angle_diff < -M_PI) {
        encoder->turns++;
    }

    if(!encoder->first_sample) {
        encoder->turns = 0;
        encoder->first_sample = 1;
        encoder->filtered_pos = encoder->angle_singleturn;
        encoder->filtered_vel = 0;
    }

    // multi-turn
    encoder->angle_multiturn[0] = encoder->angle_singleturn + TWO_PI_F*(float)encoder->turns;
    float pos_error = encoder->angle_singleturn - encoder->filtered_pos;
    if(pos_error > PI_F) pos_error -= TWO_PI_F;
    else if(pos_error < -PI_F) pos_error += TWO_PI_F;


//  float w_3db = TWO_PI_F * VELOCITY_FILTER_HZ;  // 100Hz
    float kp = PLL_KP;                            // 2.0f * 2.0f * π * 100.0f
    float ki = PLL_KI;                            // (2.0f * π * 100.0f)^2

    // Cập nhật bộ lọc PLL
    encoder->filtered_vel += dt * ki * pos_error;
    encoder->filtered_pos += dt * (encoder->filtered_vel + kp * pos_error);

    // Giới hạn vận tốc
    encoder->filtered_vel = fast_fmaxf(fast_fminf(encoder->filtered_vel, MAX_VELOCITY_RAD), -(MAX_VELOCITY_RAD));

	// Chuẩn hóa vị trí về [0, 2π]
    encoder->filtered_pos = wrap_zero_to_two_pi(encoder->filtered_pos);
    encoder->elec_angle = wrap_zero_to_two_pi(encoder->ppairs * encoder->angle_singleturn);

	// Cập nhật vận tốc điện
	encoder->velocity = encoder->filtered_vel;
	encoder->elec_velocity = encoder->ppairs * encoder->velocity;

}


void ps_print(EncoderStruct * encoder, int dt_ms){
		printf("Raw: %d\r\n", encoder->raw);
		printf("Linearized Count: %d\r\n", encoder->count);
		printf("Single Turn: %f\r\n", encoder->filtered_pos);
		printf("Speed: %f\r\r\n", encoder->filtered_vel);
		printf("Multiturn: %f\r\n", encoder->angle_multiturn[0]);
		printf("Electrical: %f\r\n", encoder->elec_angle);
		printf("Turns:  %d\r\n", encoder->turns);
	//	HAL_Delay(dt_ms);
}

void ps_validcheck(void) {

	uint16_t RAW_X_CMD = 0xC061;
	uint16_t RAW_Y_CMD = 0x8071;
	uint16_t RAW_X = 0;
	uint16_t RAW_Y = 0;

	//Check valid
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&ENC_SPI, (uint8_t*)&RAW_X_CMD, 1, 10);
	HAL_SPI_Receive(&ENC_SPI, (uint8_t*)&RAW_X, 1, 10);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET);
	HAL_Delay(1);

	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&ENC_SPI, (uint8_t*)&RAW_Y_CMD, 1, 10);
	HAL_SPI_Receive(&ENC_SPI, (uint8_t*)&RAW_Y, 1, 10);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET);
	HAL_Delay(1);

	if(RAW_X == 0 && RAW_Y == 0) {
		printf("TLE5014SP Failed! - Kill for safety");
		while(1) {
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
			HAL_Delay(100);
		}
	}
	printf("RAW_X: 0x%04X, RAW_Y: 0x%04X\r\n", RAW_X, RAW_Y);
	printf("TLE5014SP is OkVip! - Continue\r\n");
}

void ps_checkmode(void) {

	uint16_t MODE_CMD = 0xC002;
	uint16_t mode = 0;

	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&ENC_SPI, (uint8_t*)&MODE_CMD, 1, 10);
	HAL_SPI_Receive(&ENC_SPI, (uint8_t*)&mode, 1, 10);
	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET);
	printf("PWI_MD_USR Register: 0x%04X\r\n", mode);
	
}

float lerp_circular(float* array, size_t array_size, float scale, float ratio) {
    if (array_size == 0) {
        return 0.0f;
    }
    size_t left_index = (size_t)(ratio * array_size);
    left_index = (left_index < array_size) ? left_index : (array_size - 1);
    size_t right_index = (left_index + 1) % array_size;
    float fraction = (ratio - (float)left_index / array_size) * array_size;
    float left_comp = array[left_index] * scale;
    float right_comp = array[right_index] * scale;
    return (left_comp * (1.0f - fraction)) + (right_comp * fraction);
}


float wrap_zero_to_two_pi(float x) {
    int32_t divisor = (int32_t)(x / TWO_PI_F);
    float mod = x - divisor * TWO_PI_F;
    return (mod >= 0.0f) ? mod : (mod + TWO_PI_F);
}
