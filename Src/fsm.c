#include "fsm.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "user_config.h"
#include "hw_config.h"
#include "structs.h"
#include "foc.h"
#include "math_ops.h"
#include "position_sensor.h"
#include "drv8323.h"


void fsm_enter_state(FSMStruct * fsmstate){
	 /* Called when entering a new state
	  * Do necessary setup   */

		switch(fsmstate->state){
				case MENU_MODE:
				printf("\r\nEntering Main Menu\r\n");
				enter_menu_state();
				break;
			case SETUP_MODE:
				printf("\r\nEntering Setup\r\n");
				enter_setup_state();
				break;
			case ENCODER_MODE:
				printf("\r\nEntering Encoder Mode\r\n");
				break;
			case MOTOR_MODE:
				printf("\r\nEntering Motor Mode\r\n");
				enter_motor_mode();

				break;
			case CALIBRATION_MODE:
				printf("\r\nEntering Calibration Mode\r\n");
				/* zero out all calibrations before starting */
				comm_encoder_cal.done_cal = 0;
				comm_encoder_cal.done_ordering = 0;
				comm_encoder_cal.started = 0;
				comm_encoder.e_zero = 0;
				memset(&comm_encoder.offset_lut, 0, sizeof(comm_encoder.offset_lut));
				drv_enable_gd(drv);
				break;
			case PI_CURRENT_TEST:
				printf("\r\nEntering PI Current Tunning Mode\r\n");
				reset_foc(&controller);
				enter_PI_tunning_mode();
				break;
		}
 }

 void fsm_exit_state(FSMStruct * fsmstate){
	 /* Called when exiting the current state
	  * Do necessary cleanup  */

		switch(fsmstate->state){
			case MENU_MODE:
				printf("\r\nLeaving Main Menu\r\n");
				fsmstate->ready = 1;
				break;
			case SETUP_MODE:
				printf("\r\nLeaving Setup Menu\r\n");
				fsmstate->ready = 1;
				break;
			case ENCODER_MODE:
				printf("\r\nLeaving Encoder Mode\r\n");
				fsmstate->ready = 1;
				break;
			case MOTOR_MODE:
				/* Don't stop commutating if there are high currents or FW happening */
				//if( (fabs(controller.i_q_filt)<1.0f) && (fabs(controller.i_d_filt)<1.0f) ){
					fsmstate->ready = 1;
//					drv_disable_gd(drv);
//					reset_foc(&controller);
					printf("\r\nLeaving Motor Mode\r\n");
					fsmstate->bytecount = 0;
					fsmstate->cmd_id = 0;
					memset(&fsmstate->cmd_buff, 0, sizeof(fsmstate->cmd_buff));
					controller.Control_Ready = 0;
					controller.Valid_Control_Cmd = 0;
					HAL_GPIO_WritePin(LED, GPIO_PIN_RESET );
				//}
				zero_commands(&controller);		// Set commands to zero
				break;
			case CALIBRATION_MODE:
				printf("\r\nExiting Calibration Mode\r\n");
//				drv_disable_gd(drv);
				//free(error_array);
				//free(lut_array);

				fsmstate->ready = 1;
				break;

			case PI_CURRENT_TEST:
				printf("\r\nExiting PI Tuuning Mode\r\n");
				pi_control_ready = 0;
				pi_collecting_samples = 0;
				fsmstate->bytecount = 0;
				fsmstate->cmd_id = 0;
				memset(&fsmstate->cmd_buff, 0, sizeof(fsmstate->cmd_buff));
				zero_current(&controller);
				fsmstate->ready = 1;
				break;
		}

 }

void update_fsm(FSMStruct * fsmstate, char fsm_input){
	 /*update_fsm is only run when new state-change information is received
	  * on serial terminal input or CAN input
	  */
	if(fsm_input == MENU_CMD){	// escape to exit to rest mode
		fsmstate->next_state = MENU_MODE;
		fsmstate->ready = 0;
		return;
	}
	switch(fsmstate->state){
		case MENU_MODE:
			switch (fsm_input){
				case CAL_CMD:
					fsmstate->next_state = CALIBRATION_MODE;
					fsmstate->ready = 0;
					break;
				case MOTOR_CMD:
					fsmstate->next_state = MOTOR_MODE;
					fsmstate->ready = 0;
					break;
				case ENCODER_CMD:
					fsmstate->next_state = ENCODER_MODE;
					fsmstate->ready = 0;
					break;
				case SETUP_CMD:
					fsmstate->next_state = SETUP_MODE;
					fsmstate->ready = 0;
					break;
				case ZERO_CMD:
					comm_encoder.m_zero = 0;
					ps_sample(&comm_encoder, DT);
					int zero_count = comm_encoder.count;
					M_ZERO = zero_count;
					if (!preference_writer_ready(prefs)){ preference_writer_open(&prefs);}
					preference_writer_flush(&prefs);
					preference_writer_close(&prefs);
					preference_writer_load(prefs);
					printf("\r\n  Saved new zero position:  %d\r\n\r\n", M_ZERO);
					break;
				
				case PI_CURRENT_CMD:
					fsmstate->next_state = PI_CURRENT_TEST;
					fsmstate->ready = 0;
					break;
				}
			break;

		case SETUP_MODE:
			if(fsm_input == ENTER_CMD){
				process_user_input(fsmstate);
				break;
			}
			if(fsmstate->bytecount == 0){fsmstate->cmd_id = fsm_input;}
			else{
				fsmstate->cmd_buff[fsmstate->bytecount-1] = fsm_input;
				//fsmstate->bytecount = fsmstate->bytecount%(sizeof(fsmstate->cmd_buff)/sizeof(fsmstate->cmd_buff[0])); // reset when buffer is full
			}
			fsmstate->bytecount++;
			/* If enter is typed, process user input */

			break;

		case ENCODER_MODE:
			break;

		case MOTOR_MODE:
			if(fsm_input == ENTER_CMD){
				parse_motor_command(fsmstate);
				break;
			}

			fsmstate->cmd_buff[fsmstate->bytecount] = fsm_input;
			fsmstate->bytecount++;
			break;

		case PI_CURRENT_TEST:
			if(fsm_input == ENTER_CMD){
				parse_pi_current_command(fsmstate);
				break;
			}

			fsmstate->cmd_buff[fsmstate->bytecount] = fsm_input;
			fsmstate->bytecount++;
			break;

	}
	//printf("FSM State: %d  %d\r\n", fsmstate.state, fsmstate.state_change);
 }


 void enter_menu_state(void){
	    //drv.disable_gd();
	    //reset_foc(&controller);
	    //gpio.enable->write(0);
	 	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	    printf("\r\n\r\n");
	    printf(" Commands:\r\n");
	    printf(" m - Motor Mode\r\n");
	    printf(" c - Calibrate Encoder\r\n");
	    printf(" s - Setup\r\n");
	    printf(" e - Display Encoder\r\n");
	    printf(" z - Set Zero Position\r\n");
		printf(" i - PI Current tunning\r\n");
	    printf(" esc - Exit to Menu\r\n");
	    //gpio.led->write(0);
 }

 void enter_setup_state(void){
	    printf("\r\n Configuration Options \r\n");
	    printf(" %-4s %-31s %-5s %-6s %-2s\r\n", "prefix", "parameter", "min", "max", "current value");
	    printf("\r\n Motor:\r\n");
	    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "g", "Gear Ratio", "0", "-", GR);
	    printf(" %-4s %-31s %-5s %-6s %.5f\r\n", "k", "Torque Constant (N-m/A)", "0", "-", KT);
	    printf("\r\n Control:\r\n");
	    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "b", "Current Bandwidth (Hz)", "100", "2000", I_BW);
	    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "l", "Current Limit (A)", "0.0", "75.0", I_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "p", "Max Position Setpoint (rad)", "-", "-", P_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "v", "Max Velocity Setpoint (rad)/s", "-", "-", V_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "x", "Max Position Gain (N-m/rad)", "0.0", "1000.0", KP_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "d", "Max Velocity Gain (N-m/rad/s)", "0.0", "5.0", KD_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "f", "FW Current Limit (A)", "0.0", "33.0", I_FW_MAX);
	    //printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "h", "Temp Cutoff (C) (0 = none)", "0", "150", TEMP_MAX);
	    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "c", "Continuous Current (A)", "0.0", "40.0", I_MAX_CONT);
	    printf(" %-4s %-31s %-5s %-6s %.3f\r\n", "a", "Calibration Current (A)", "0.0", "20.0", I_CAL);
	    printf(" \n\r To change a value, type 'prefix''value''ENTER'\r\n e.g. 'b1000''ENTER'\r\n ");
	    printf("VALUES NOT ACTIVE UNTIL POWER CYCLE! \r\n\r\n");
 }

 void process_user_input(FSMStruct * fsmstate){
	 /* Collects user input from serial (maybe eventually CAN) and updates settings */

	 switch (fsmstate->cmd_id){
		 case 'b':
			 I_BW = fmaxf(fminf(atof(fsmstate->cmd_buff), 2000.0f), 100.0f);
			 printf("I_BW set to %f\r\n", I_BW);
			 break;
		 case 'l':
			 I_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 75.0f), 0.0f);
			 printf("I_MAX set to %f\r\n", I_MAX);
			 break;
		 case 'f':
			 I_FW_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 33.0f), 0.0f);
			 printf("I_FW_MAX set to %f\r\n", I_FW_MAX);
			 break;
		 case 'h':
			 TEMP_MAX = fmaxf(fminf(atof(fsmstate->cmd_buff), 150.0f), 0.0f);
			 printf("TEMP_MAX set to %f\r\n", TEMP_MAX);
			 break;
		 case 'c':
			 I_MAX_CONT = fmaxf(fminf(atof(fsmstate->cmd_buff), 40.0f), 0.0f);
			 printf("I_MAX_CONT set to %f\r\n", I_MAX_CONT);
			 break;
		 case 'a':
			 I_CAL = fmaxf(fminf(atof(fsmstate->cmd_buff), 20.0f), 0.0f);
			 printf("I_CAL set to %f\r\n", I_CAL);
			 break;
		 case 'g':
			 GR = fmaxf(atof(fsmstate->cmd_buff), .001f);	// Limit prevents divide by zero if user tries to enter zero
			 printf("GR set to %f\r\n", GR);
			 break;
		 case 'k':
			 KT = fmaxf(atof(fsmstate->cmd_buff), 0.0001f);	// Limit prevents divide by zero.  Seems like a reasonable LB?
			 printf("KT set to %f\r\n", KT);
			 break;
		 case 'x':
			 KP_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 printf("KP_MAX set to %f\r\n", KP_MAX);
			 break;
		 case 'd':
			 KD_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 printf("KD_MAX set to %f\r\n", KD_MAX);
			 break;
		 case 'p':
			 P_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 P_MIN = -P_MAX;
			 printf("P_MAX set to %f\r\n", P_MAX);
			 break;
		 case 'v':
			 V_MAX = fmaxf(atof(fsmstate->cmd_buff), 0.0f);
			 V_MIN = -V_MAX;
			 printf("V_MAX set to %f\r\n", V_MAX);
			 break;
		 default:
			 printf("\r\n '%c' Not a valid command prefix\r\n\r\n", fsmstate->cmd_buff);
			 break;

		 }

	 /* Write new settings to flash */

	 if (!preference_writer_ready(prefs)){ preference_writer_open(&prefs);}
	 preference_writer_flush(&prefs);
	 preference_writer_close(&prefs);
	 preference_writer_load(prefs);

	 enter_setup_state();

	 fsmstate->bytecount = 0;
	 fsmstate->cmd_id = 0;
	 memset(&fsmstate->cmd_buff, 0, sizeof(fsmstate->cmd_buff));
 }

 void enter_motor_mode(void){
     printf("\r\nEnter control commands for the motor in the following format:\r\n");
     printf("[p_des],[v_des],[mode],ENTER\r\n");
     printf("Example: 1.5,2.0,1,ENTER\r\n");
     printf("Mode: 0 - Step Velocity, 1 - S-Curved, 2 - Sine, 3 - LSPB\r\n");
     printf("After entering, press ENTER to confirm\r\n");
 }

 void enter_PI_tunning_mode(void){
     printf("\r\nEnter control commands for the motor in the following format:\r\n");
     printf("[kp],[ki],[id_des],[ENTER]\r\n");
     printf("Example: 32.01,628.05,2.0,ENTER\r\n");
     printf("After entering, press ENTER to confirm\r\n");
 }

void parse_motor_command(FSMStruct * fsmstate) {
    // Debug: Print received data
    printf("Received buffer: %s\r\n", fsmstate->cmd_buff);
    
    // Ensure buffer is null-terminated
    fsmstate->cmd_buff[sizeof(fsmstate->cmd_buff)] = '\0';
    
    // Parse command with format "p_des,v_des,mode,ENTER"
    char *token = strtok(fsmstate->cmd_buff, ",");
    if (token == NULL) {
        printf("Error: Invalid format. Need: p_des,v_des,mode\r\n");
        goto cleanup;
    }
    
    // Parse p_des
    float p_des = atof(token);
    if (p_des == 0.0f && token[0] != '0') {
        printf("Error: Invalid p_des value\r\n");
        goto cleanup;
    }
    
    // Parse v_des
    token = strtok(NULL, ",");
    if (token == NULL) {
        printf("Error: Missing v_des parameter\r\n");
        goto cleanup;
    }
    float v_des = atof(token);
    if (v_des == 0.0f && token[0] != '0') {
        printf("Error: Invalid v_des value\r\n");
        goto cleanup;
    }
    
    // Parse mode
    token = strtok(NULL, ",");
    if (token == NULL) {
        printf("Error: Missing mode parameter\r\n");
        goto cleanup;
    }
    int mode = atoi(token);
    if (mode < 0 || mode > 3) {
        printf("Error: Invalid mode value (must be 0, 1, 2 or 3)\r\n");
        goto cleanup;
    }
    
    // Check for extra parameters
    token = strtok(NULL, ",");
    if (token != NULL) {
        printf("Warning: Extra parameters found, using only first three\r\n");
    }
    
    // Update controller parameters
    controller.commands[0] = p_des;    // Position desired
    controller.commands[1] = v_des;    // Velocity desired
//    controller.commands[5] = mode;     // Control mode
    controller.plan_mode = (int)mode;

    if(p_des - comm_encoder.angle_multiturn[0] < 0) {
    	controller.v_des = -v_des;
    }
    controller.timeout = 0;
    controller.Valid_Control_Cmd = 1;
    
    printf("Motor command set: P_des: %.2f, V_des: %.2f, Mode: %d\r\n",
           p_des, v_des, mode);
    fsmstate->bytecount = 0;
    fsmstate->cmd_id = 0;
    memset(&fsmstate->cmd_buff, 0, sizeof(fsmstate->cmd_buff));
    return;
    
cleanup:
    // Clean up buffer and reset counters
    fsmstate->bytecount = 0;
    fsmstate->cmd_id = 0;
    memset(&fsmstate->cmd_buff, 0, sizeof(fsmstate->cmd_buff));
    printf("Error: Need 3 parameters: p_des,v_des,mode\r\n");
}

void parse_pi_current_command(FSMStruct * fsmstate) {
    // Debug: Print received data
    printf("Received buffer: %s\r\n", fsmstate->cmd_buff);
    
    // Ensure buffer is null-terminated
//    fsmstate->cmd_buff[sizeof(fsmstate->cmd_buff) - 1] = '\0';
    fsmstate->cmd_buff[sizeof(fsmstate->cmd_buff)] = '\0';
    
    // Parse command with format "kp,ki,id_des,ENTER"
    char *token = strtok(fsmstate->cmd_buff, ",");
    if (token == NULL) {
        printf("Error: Invalid format. Need: kp,ki,id_des\r\n");
        goto cleanup;
    }
    
    // Parse kp
    float kp = atof(token);
    if (kp == 0.0f && token[0] != '0') {
        printf("Error: Invalid kp value\r\n");
        goto cleanup;
    }
    
    // Parse ki
    token = strtok(NULL, ",");
    if (token == NULL) {
        printf("Error: Missing ki parameter\r\n");
        goto cleanup;
    }
    float ki = atof(token);
    if (ki == 0.0f && token[0] != '0') {
        printf("Error: Invalid ki value\r\n");
        goto cleanup;
    }
    
    // Parse id_des
    token = strtok(NULL, ",");
    if (token == NULL) {
        printf("Error: Missing id_des parameter\r\n");
        goto cleanup;
    }
    float id_des = atof(token);
    if (id_des == 0.0f && token[0] != '0') {
        printf("Error: Invalid id_des value\r\n");
        goto cleanup;
    }
    
    // Check for extra parameters
    token = strtok(NULL, ",");
    if (token != NULL) {
        printf("Warning: Extra parameters found, using only first three\r\n");
    }
    
    // Update controller parameters
    controller.k_d = kp;
    controller.ki_d = ki;
    controller.i_d_des = id_des;
    controller.i_q_des = 0;
    pi_control_ready = 0;
    pi_collecting_samples = 0;
    PI_Print_Ready = 0;
    pi_sample_count = 0;
    
    printf("PI Current Test: Kp: %.2f, Ki: %.2f, Id_des: %.2f\r\n", kp, ki, id_des);
	fsmstate->bytecount = 0;
	fsmstate->cmd_id = 0;
	memset(&fsmstate->cmd_buff, 0, sizeof(fsmstate->cmd_buff));
    return;
    
cleanup:
    // Clean up buffer and reset counters
	fsmstate->bytecount = 0;
	fsmstate->cmd_id = 0;
	memset(&fsmstate->cmd_buff, 0, sizeof(fsmstate->cmd_buff));
    printf("Error: Need 3 parameters: kp,ki,id_des\r\n");
}
