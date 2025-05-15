
#ifndef INC_POSITION_SENSOR_H_
#define INC_POSITION_SENSOR_H_


//#include "structs.h"
#include "spi.h"
#include <stdint.h>

#define N_POS_SAMPLES 20		// Number of position samples to store.  should put this somewhere else...
#define N_LUT 128

//typedef struct{
//	union{
//		uint8_t spi_tx_buff[2];
//		uint16_t spi_tx_word;
//	};
//	union{
//		uint8_t spi_rx_buff[2];
//		uint16_t spi_rx_word;
//	};
//	float angle_singleturn, old_angle, angle_multiturn[N_POS_SAMPLES], elec_angle, velocity, elec_velocity, ppairs, vel2;
//	float output_angle_multiturn;
//	int raw, vel_raw, count, old_count, turns;
//	int count_buff[N_POS_SAMPLES];
//	int m_zero, e_zero;
//	int offset_lut[N_LUT];
//	uint8_t first_sample;
//} EncoderStruct;

typedef struct {
    // SPI communication
    union {
        uint8_t spi_tx_buff[2];
        uint16_t spi_tx_word;
    };
    union {
        uint8_t spi_rx_buff[2];
        uint16_t spi_rx_word;
    };

    // Position tracking
    float angle_singleturn;     // Current angle in radians [-π to π]
    float old_angle;            // Previous angle
    float angle_multiturn[N_POS_SAMPLES];  // Multi-turn angle history
    float elec_angle;           // Electrical angle [0 to 2π]
    float velocity;             // Mechanical velocity in rad/s
    float elec_velocity;        // Electrical velocity in rad/s
    float ppairs;               // Number of pole pairs
    float vel2;                 // Alternative velocity calculation
    float sin_theta_e;
    float cos_theta_e;

    // PLL filter variables
    float filtered_pos;         // Filtered position in radians
    float filtered_vel;         // Filtered velocity in rad/s
    float time_since_update;    // Time since last update

    // Raw encoder values
    int raw;                    // Raw encoder reading (15-bit)
    int vel_raw;               // Raw velocity reading
    int count;                 // Linearized count
    int old_count;            // Previous count
    int turns;                // Number of full rotations
    float raw_velocity;      // Vận tốc thô chưa lọc (rad/s)
    // Calibration
    int count_buff[N_POS_SAMPLES];
    int m_zero;               // Mechanical zero position
    int e_zero;              // Electrical zero position
    int offset_lut[N_LUT];   // Linearization lookup table
    int offset_elec;
    float mecha_pos_ramp, elec_pos_ramp;

    uint8_t first_sample;     // First sample flag
} EncoderStruct;




#ifdef __cplusplus
extern "C" {
#endif

void ps_warmup(EncoderStruct * encoder, int n);
void ps_sample(EncoderStruct * encoder, float dt);
void ps_print(EncoderStruct * encoder, int dt_ms);
void ps_validcheck(void);
void ps_checkmode(void);


float lerp_circular(float* array, size_t array_size, float scale, float ratio);
float wrap_zero_to_two_pi(float x);

#ifdef __cplusplus
}
#endif


#endif /* INC_POSITION_SENSOR_H_ */
