#ifndef INC_FSM_H_
#define INC_FSM_H_
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "hw_config.h"


#define MENU_MODE           0
#define CALIBRATION_MODE    1
#define MOTOR_MODE          2
#define SETUP_MODE          4
#define ENCODER_MODE        5
#define INIT_TEMP_MODE      6
#define PI_CURRENT_TEST		7



#define MENU_CMD			27
#define MOTOR_CMD			'm'
#define CAL_CMD				'c'
#define ENCODER_CMD			'e'
#define SETUP_CMD			's'
#define ZERO_CMD			'z'
#define PI_CURRENT_CMD		'i'
#define ENTER_CMD			13


extern float id_samples[MAX_SAMPLES];
extern float iq_samples[MAX_SAMPLES];
extern float pi_t_samples[MAX_SAMPLES];
extern uint32_t pi_sample_count;
extern uint8_t pi_collecting_samples;
extern uint8_t PI_Print_Ready;
extern uint8_t pi_control_ready;


typedef struct{
	uint8_t state;
	uint8_t next_state;
	uint8_t state_change;
	uint8_t ready;
	uint8_t Received;

	char cmd_buff[20];
	char bytecount;
	char cmd_id;

}FSMStruct;


void run_fsm(FSMStruct* fsmstate);
void update_fsm(FSMStruct * fsmstate, char fsm_input);
void fsm_enter_state(FSMStruct * fsmstate);
void fsm_exit_state(FSMStruct * fsmstate);
void enter_menu_state(void);
void enter_setup_state(void);
void enter_motor_mode(void);
void process_user_input(FSMStruct * fsmstate);
void parse_motor_command(FSMStruct * fsmstate);
void enter_PI_tunning_mode(void);
void parse_pi_current_command(FSMStruct * fsmstate);
#ifdef __cplusplus
}
#endif

#endif /* INC_FSM_H_ */
