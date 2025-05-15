#include "foc.h"
#include "adc.h"
#include "tim.h"
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"

void set_dtc(ControllerStruct *controller){

	/* Invert duty cycle if that's how hardware is configured */
	float dtc_u = fast_fmaxf(fast_fminf(controller->dtc_u, DTC_MAX), DTC_MIN);
	float dtc_v = fast_fmaxf(fast_fminf(controller->dtc_v, DTC_MAX), DTC_MIN);
	float dtc_w = fast_fmaxf(fast_fminf(controller->dtc_w, DTC_MAX), DTC_MIN);

	if(INVERT_DTC){
		dtc_u = 1.0f - controller->dtc_u;
		dtc_v = 1.0f - controller->dtc_v;
		dtc_w = 1.0f - controller->dtc_w;
	}
	/* Handle phase order swapping so that voltage/current/torque match encoder direction */
	if(!PHASE_ORDER){
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_U, ((TIM_PWM.Instance->ARR))*dtc_u);
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_V, ((TIM_PWM.Instance->ARR))*dtc_v);
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_W, ((TIM_PWM.Instance->ARR))*dtc_w);

		HAL_GPIO_WritePin(DRV_HIZ, 1);

	}
	else{
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_V, ((TIM_PWM.Instance->ARR))*dtc_u);
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_U, ((TIM_PWM.Instance->ARR))*dtc_v);
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_W, ((TIM_PWM.Instance->ARR))*dtc_w);

		HAL_GPIO_WritePin(DRV_HIZ, 1);
	}
}

void analog_sample (ControllerStruct *controller){

	HAL_ADC_Start(&ADC_CH_IA);
	HAL_ADC_Start(&ADC_CH_IB);
	HAL_ADC_Start(&ADC_CH_IC);

	if(controller->phase_order == 0){

		HAL_ADC_PollForConversion(&ADC_CH_IA, 1);
		controller->adc_a_raw = HAL_ADC_GetValue(&ADC_CH_IA);

		HAL_ADC_PollForConversion(&ADC_CH_IB, 1);
		controller->adc_b_raw = HAL_ADC_GetValue(&ADC_CH_IB);

		HAL_ADC_PollForConversion(&ADC_CH_IC, 1);
		controller->adc_c_raw = HAL_ADC_GetValue(&ADC_CH_IC);

		HAL_ADC_PollForConversion(&ADC_CH_VBUS, 1);
		controller->adc_vbus_raw = HAL_ADC_GetValue(&ADC_CH_VBUS);

	}
	else {

		HAL_ADC_PollForConversion(&ADC_CH_IA, 1);
		controller->adc_b_raw = HAL_ADC_GetValue(&ADC_CH_IA);

		HAL_ADC_PollForConversion(&ADC_CH_IB, 1);
		controller->adc_a_raw = HAL_ADC_GetValue(&ADC_CH_IB);

		HAL_ADC_PollForConversion(&ADC_CH_IC, 1);
		controller->adc_c_raw = HAL_ADC_GetValue(&ADC_CH_IC);

		HAL_ADC_PollForConversion(&ADC_CH_VBUS, 1);
		controller->adc_vbus_raw = HAL_ADC_GetValue(&ADC_CH_VBUS);

	}

	controller->i_a =  (float)(controller->adc_a_offset - controller->adc_a_raw)*I_SCALE;
	controller->i_c =  (float)(controller->adc_b_offset - controller->adc_b_raw)*I_SCALE;
	controller->i_b =  (float)(controller->adc_c_offset - controller->adc_c_raw)*I_SCALE;

	controller->v_bus = (float)controller->adc_vbus_raw*V_SCALE;
	ExponentialFilter(controller,0.001);
}

void abc( float theta, float d, float q, float *a, float *b, float *c){
    /* Inverse DQ0 Transform
    Phase current amplitude = lengh of dq vector
    i.e. iq = 1, id = 0, peak phase current of 1 */

    float cf = cos_lut(theta);
    float sf = sin_lut(theta);

    *a = cf*d - sf*q;
    *b = (SQRT3_2*sf-.5f*cf)*d - (-SQRT3_2*cf-.5f*sf)*q;
    *c = (-SQRT3_2*sf-.5f*cf)*d - (SQRT3_2*cf-.5f*sf)*q;
}


void dq0(float theta, float a, float b, float c, float *d, float *q){
    /* DQ0 Transform
    Phase current amplitude = lengh of dq vector
    i.e. iq = 1, id = 0, peak phase current of 1*/

    float cf = cos_lut(theta);
    float sf = sin_lut(theta);

    *d = 0.6666667f*(cf*a + (SQRT3_2*sf-.5f*cf)*b + (-SQRT3_2*sf-.5f*cf)*c);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-sf*a - (-SQRT3_2*cf-.5f*sf)*b - (SQRT3_2*cf-.5f*sf)*c);


}

void svm(float v_max, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w){
    /* Space Vector Modulation
     u,v,w amplitude = v_bus for full modulation depth */

    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w))*0.5f;
    float v_midpoint = .5f*(DTC_MAX+DTC_MIN);

    *dtc_u = fast_fminf(fast_fmaxf((.5f*(u -v_offset)*OVERMODULATION/v_max + v_midpoint ), DTC_MIN), DTC_MAX);
    *dtc_v = fast_fminf(fast_fmaxf((.5f*(v -v_offset)*OVERMODULATION/v_max + v_midpoint ), DTC_MIN), DTC_MAX);
    *dtc_w = fast_fminf(fast_fmaxf((.5f*(w -v_offset)*OVERMODULATION/v_max + v_midpoint ), DTC_MIN), DTC_MAX);

}

void zero_current(ControllerStruct *controller){
	
	float dtc_u = 0;
	float dtc_v = 0;
	float dtc_w = 0;

    controller->dtc_u = 0.f;
    controller->dtc_v = 0.f;
    controller->dtc_w = 0.f;

	if(INVERT_DTC){
		dtc_u = 1.0f - controller->dtc_u;
		dtc_v = 1.0f - controller->dtc_v;
		dtc_w = 1.0f - controller->dtc_w;
	}
	/* Handle phase order swapping so that voltage/current/torque match encoder direction */
	if(!PHASE_ORDER){
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_U, ((TIM_PWM.Instance->ARR))*dtc_u);
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_V, ((TIM_PWM.Instance->ARR))*dtc_v);
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_W, ((TIM_PWM.Instance->ARR))*dtc_w);
		HAL_GPIO_WritePin(DRV_HIZ, 0);

	}
	else{
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_V, ((TIM_PWM.Instance->ARR))*dtc_u);
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_U, ((TIM_PWM.Instance->ARR))*dtc_v);
		__HAL_TIM_SET_COMPARE(&TIM_PWM, TIM_CH_W, ((TIM_PWM.Instance->ARR))*dtc_w);
		HAL_GPIO_WritePin(DRV_HIZ, 0);
	}

}

void offset_current(ControllerStruct *controller){
	/* Measure zero-current ADC offset */

    int adc_a_offset = 0;
    int adc_b_offset = 0;
    int adc_c_offset = 0;

    int n = 1000;
    zero_current(controller);

    for (int i = 0; i<n; i++){               // Average n samples
    	analog_sample(controller);
    	adc_a_offset +=  controller->adc_a_raw;
    	adc_b_offset += controller->adc_b_raw;
    	adc_c_offset += controller->adc_c_raw;
     }
    controller->adc_a_offset = adc_a_offset/n;
    controller->adc_b_offset = adc_b_offset/n;
    controller->adc_c_offset = adc_c_offset/n;

}



void init_controller_params(ControllerStruct *controller){
	//Todo: Change parameter
	//Init PI FOC

	//KI = w3dB*R;
  controller->ki_d = KI_D;
  controller->ki_q = KI_Q;
    //KP = w3dB*L;

  controller->k_d = K_D;
  controller->k_q = K_Q;

    //Low pass filter
//    controller->alpha = 1.0f - 1.0f/(1.0f - DT*I_BW*TWO_PI_F);
	controller->alpha = 0.001;
    //KI FeedForward
    controller->rate_hz = 1/DT;
    controller->phase_order = PHASE_ORDER;
    controller->flux_linkage = KT/(1.5f*PPAIRS);

}

void reset_foc(ControllerStruct *controller){

//	TIM_PWM.Instance->CCR3 = ((TIM_PWM.Instance->ARR))*(0.5f);
//	TIM_PWM.Instance->CCR1 = ((TIM_PWM.Instance->ARR))*(0.5f);
//	TIM_PWM.Instance->CCR2 = ((TIM_PWM.Instance->ARR))*(0.5f);
	zero_current(controller);
    controller->i_d_des = 0;
    controller->i_q_des = 0;
    controller->i_d = 0;
    controller->i_q = 0;
    controller->i_q_filt = 0;
    controller->q_int = 0;
    controller->d_int = 0;
    controller->v_q = 0;
    controller->v_d = 0;
    controller->fw_int = 0;
    controller->otw_flag = 0;

    }


float linearize_dtc(ControllerStruct *controller, float dtc)
{
    float duty = fast_fmaxf(fast_fminf(fabs(dtc), .999f), 0.0f);;
    int index = (int) (duty*127.0f);
    float val1 = controller->inverter_tab[index];
    float val2 = controller->inverter_tab[index+1];
    return val1 + (val2 - val1)*(duty*128.0f - (float)index);
}

//Field Weakening for I-PMSM
void field_weaken(ControllerStruct *controller)
{
   /// Field Weakening ///

   controller->fw_int += controller->ki_fw*(controller->v_max - 1.0f - controller->v_ref);
   controller->fw_int = fast_fmaxf(fast_fminf(controller->fw_int, 0.0f), -I_FW_MAX);
   controller->i_q_des = controller->i_q_des + (controller->i_q_des > 0)*controller->fw_int + (controller->i_q_des < 0)*controller->fw_int;
   controller->i_d_des = controller->fw_int;
   float q_max = sqrtf(controller->i_max*controller->i_max - controller->i_d_des*controller->i_d_des);
   controller->i_q_des = fast_fmaxf(fast_fminf(controller->i_q_des, q_max), -q_max);
}

//FOC and SVPWM

void FOC(ControllerStruct *controller, EncoderStruct *encoder)
{
//    controller->theta_elec = encoder->elec_angle;
//    controller->dtheta_elec = encoder->elec_velocity;
//    controller->dtheta_mech = encoder->filtered_vel/GR;
//    controller->theta_mech = encoder->angle_multiturn[0]/GR;

    //Overmodulation and voltage limits
    controller->v_max = OVERMODULATION*controller->v_bus_filt*(DTC_MAX-DTC_MIN)*SQRT1_3;
    controller->i_max = 25;

    // Bù cogging torque
    float commutation_position = encoder->angle_singleturn/TWO_PI_F;
    float q_comp_A = lerp_circular(cogging_lut_1024, COGGING_TABLE_SIZE,1,commutation_position) * COGGING_SCALE;
    controller->i_q_des += q_comp_A;

    controller->i_d_des = fast_fmaxf(fast_fminf(controller->i_d_des, controller->i_max), -(controller->i_max));
    controller->i_q_des = fast_fmaxf(fast_fminf(controller->i_q_des, controller->i_max), -(controller->i_max));
//    limit_norm(&controller->i_d_des, &controller->i_q_des, controller->i_max);

    // PI Currents Controller with anti-windup //
    controller->d_err = controller->i_d_des - controller->i_d;
    controller->d_int += controller->d_err*controller->ki_d*DT;

    controller->v_d = controller->k_d*controller->d_err + controller->d_int;

    // Anti-windup - Back Calculation
    controller->max_current_integral = MAX_VOLTAGE_RATIO * OVERMODULATION * 0.5f * controller->v_bus_filt;
    // Limit integral for PI d-axis
    controller->d_int = fast_fmaxf(fast_fminf(controller->d_int,
    		controller->max_current_integral), -(controller->max_current_integral));


    controller->q_err =  controller->i_q_des - controller->i_q;

	controller->q_int += controller->q_err*controller->ki_q*DT;

	controller->v_q = controller->k_q*controller->q_err + controller->q_int;

	// Limit integral for PI q-axis
	controller->q_int = fast_fmaxf(fast_fminf(controller->q_int,
			controller->max_current_integral), -(controller->max_current_integral));

	SVPWM_v2(controller, encoder);
}


float PID_Torque(ControllerStruct* controller, EncoderStruct* encoder) {
	//Check moteus
	float i_rate_limit = 1;
	float i_limit = 0;

	controller->pos_err = controller->p_des - controller->theta_mech;
	float error = controller->pos_err;
	float error_rate = controller->v_des - controller->dtheta_mech;
	float max_i_update = i_rate_limit*DT;
	float to_update_i = error*controller->ki*DT;
	to_update_i = fast_fmaxf(fast_fminf(to_update_i, max_i_update), -max_i_update);
	controller->pos_int += to_update_i;

//	controller->pos_int = fast_fmaxf(fast_fminf(controller->pos_int, i_limit), -i_limit);

	float p = controller->kp*error;
	float d = controller->kd*error_rate;
	float pd = p + d;

	return pd + controller->pos_int;
}

void zero_commands(ControllerStruct * controller){
	controller->t_ff = 0;
	controller->kp = 0;
	controller->kd = 0;
	controller->p_des = 0;
	controller->v_des = 0;
	controller->i_q_des = 0;
}


void ExponentialFilter(ControllerStruct * controller, float alpha) {
    if (controller->v_bus_filt == 0) {
    	controller->v_bus_filt = controller->v_bus;
    } else {
        controller->v_bus_filt = alpha * controller->v_bus + (1.0f - alpha) * controller->v_bus_filt;
    }
}



void SVPWM_v2(ControllerStruct* controller, EncoderStruct* encoder) {
	float max_voltage = (0.5f - DTC_MIN)*controller->v_bus_filt*OVERMODULATION;

	//Add small theta into theta_electrical - Just use when testing SVPWM, remove later
//	encoder->elec_angle = encoder->elec_angle + 0.5f*DT*encoder->elec_velocity;

	encoder->sin_theta_e = sin_lut(encoder->elec_angle);
	encoder->cos_theta_e = cos_lut(encoder->elec_angle);

	//Ramp test for calib
//	encoder->sin_theta_e = sin(encoder->elec_pos_ramp);
//	encoder->cos_theta_e = cos(encoder->elec_pos_ramp);

	//Limit Vd Vq
	controller->v_d = fast_fmaxf(fast_fminf(controller->v_d, max_voltage), -max_voltage);
	controller->v_q = fast_fmaxf(fast_fminf(controller->v_q, max_voltage), -max_voltage);
//	limit_norm(&controller->v_d, &controller->v_q, max_voltage);

	//Vdq -> Vabc
    controller->v_u = encoder->cos_theta_e*controller->v_d - encoder->sin_theta_e*controller->v_q;
    controller->v_v = (SQRT3_2*encoder->sin_theta_e -0.5f*encoder->cos_theta_e)*controller->v_d
    					- (-SQRT3_2*encoder->cos_theta_e -0.5f*encoder->sin_theta_e)*controller->v_q;
    controller->v_w = (-SQRT3_2*encoder->sin_theta_e -0.5f*encoder->cos_theta_e)*controller->v_d
    					- (SQRT3_2*encoder->cos_theta_e -0.5f*encoder->sin_theta_e)*controller->v_q;


    float a,b,c;
    float shift;
	//Find max-min voltage
    if(controller->v_u <= controller->v_v && controller->v_u <= controller->v_w) {
    	// u,v,w
    	a = controller->v_u;
    	b = controller->v_v;
    	c = controller->v_w;
    	shift = 0;
    }
    else if(controller->v_v <= controller->v_u && controller->v_v <= controller->v_w) {
    	// v,w,u
    	a = controller->v_v;
    	b = controller->v_w;
    	c = controller->v_u;
    	shift = 1;
    }
    else {
    	// w,u,v
    	a = controller->v_w;
    	b = controller->v_u;
    	c = controller->v_v;
    	shift = 2;
    }

    Cal_PWM_Val(controller,a,b,c,shift);

    set_dtc(controller);
}

void Cal_PWM_Val(ControllerStruct* controller, float a, float b, float c, int shift) {
	//Make sure a is minimum voltage
	float db = b - a;
	float dc = c - a;

	float fdb = db/controller->v_bus_filt;
	float fdc = dc/controller->v_bus_filt;

	float dpb = Scale_fdx(fdb,fdc);
	float dpc = Scale_fdx(fdc,fdb);

	float extent =  0.5f * fast_fmaxf(dpb,dpc);
    float pwm1 = 0.5f - extent;
    float pwm2 = pwm1 + dpb;
    float pwm3 = pwm1 + dpc;

    if (shift == 0) {
      controller->dtc_u = pwm1;
      controller->dtc_v = pwm2;
      controller->dtc_w = pwm3;
    } else if (shift == 1) {
        controller->dtc_u = pwm3;
        controller->dtc_v = pwm1;
        controller->dtc_w = pwm2;
    }
    else {
        controller->dtc_u = pwm2;
        controller->dtc_v = pwm3;
        controller->dtc_w = pwm1;
    }
}

float Scale_fdx(float fdx, float fd_other) {
	float scaled;
	if (fdx < BLEND_MIN*fd_other) {
		return fdx;
	}
	scaled = Cal_Scale(fdx);
	if (fdx < BLEND_MAX*fd_other) {
		float frac = (fdx - BLEND_MIN * fd_other) /
                (BLEND_REGION * fd_other);
		return fdx + frac * (scaled - fdx);
	}
	return scaled;


}

float Cal_Scale(float val) {
	float sign = val < 0.0f ? -1.0f : 1.0f;
	if(fabs(val) < COMP_MAG) {
		return val/COMP_MAG*COMP_OFF;
	}
	else {
	    return sign * ((0.5f - COMP_OFF) * (fabs(val) - COMP_MAG) / (0.5f - COMP_MAG) + COMP_OFF);
	}
}

float cogging_lut_1024[1024] = {
    0.06994583, 0.16870417, 0.42572083, 0.65262857, 0.66077381, 0.63777619, 0.48857500, 0.38535333,
    0.33962500, 0.27912500, 0.24966333, 0.25953750, 0.27650667, 0.34130000, 0.24858667, 0.33193333,
    0.28086750, 0.08322000, -0.04769583, -0.14132750, -0.29844167, -0.48417333, -0.53237857, -0.47526667,
    -0.36716000, -0.22956667, -0.13686250, 0.00645833, -0.02704583, -0.07921250, -0.21466250, -0.33364167,
    -0.37825333, -0.66288214, -0.63719048, -0.67840000, -0.62618571, -0.60382500, -0.52483333, -0.51093000,
    -0.55119000, -0.53352500, -0.58076667, -0.56795000, -0.56205000, -0.56840000, -0.50447500, -0.46350000,
    -0.31480417, -0.28022000, -0.14922917, 0.08256667, 0.06853750, 0.15750417, 0.11455667, 0.27294167,
    0.17735000, -0.08816667, 0.24147500, -0.03713333, 0.15875417, 0.18192917, 0.36651250, 0.47990833,
    0.63757500, 0.87472500, 0.81025417, 0.72440714, 0.78223095, 0.65377143, 0.66174643, 0.61106667,
    0.69917500, 0.66393214, 0.79172708, 0.97329048, 1.03587727, 1.01460227, 0.80506029, 0.71335385,
    0.39678929, 0.22027083, -0.04922500, -0.04930750, -0.05553667, 0.05021250, -0.06947083, 0.31460000,
    0.33194667, 0.40834167, 0.42787667, 0.23308750, 0.10222500, -0.11287500, -0.23747500, -0.38171000,
    -0.43946667, -0.42698333, -0.33548000, -0.30327917, -0.30850667, -0.31242500, -0.33376667, -0.41281667,
    -0.61315000, -0.49395000, -0.78725000, -0.83101875, -0.66995000, -0.66421500, -0.48225417, -0.32021667,
    -0.17339583, -0.16157500, 0.00403333, -0.26067917, -0.28921000, -0.43934667, -0.49709333, -0.63393333,
    -0.58016667, -0.74233333, -0.57766333, -0.47089667, -0.40463750, -0.26898750, -0.19925000, -0.11276250,
    -0.20665000, -0.18501667, -0.06291667, 0.04925417, 0.00895000, 0.10121667, 0.39540417, 0.22480000,
    0.54959667, 0.67034000, 0.55350833, 0.74244048, 0.64377857, 0.68570476, 0.57285000, 0.41679667,
    0.29506333, 0.24795417, 0.38964583, 0.40507667, 0.57664333, 0.58200833, 0.80726667, 0.74160714,
    0.71247500, 0.68326667, 0.61117500, 0.50774167, 0.50443333, 0.40803667, 0.36347917, 0.48044333,
    0.47738333, 0.50315476, 0.51990000, 0.42119762, 0.28664167, 0.08901500, -0.24674000, -0.46652619,
    -0.70008182, -0.87656667, -0.91639808, -0.80116875, -0.68844333, -0.46533750, -0.34238750, -0.25575000,
    -0.28717083, -0.38140333, -0.40547500, -0.59280833, -0.62434286, -0.71919524, -0.65548214, -0.59335833,
    -0.35678333, -0.35556250, -0.35387667, -0.40325000, -0.48982667, -0.58591667, -0.61001250, -0.68400833,
    -0.82931212, -0.76455556, -0.47159667, -0.24517333, -0.17088333, -0.00433333, 0.18661250, 0.28422917,
    0.26825833, 0.25071250, 0.03455833, 0.19577500, 0.04367500, 0.07270833, 0.06929583, 0.11275000,
    0.15790000, 0.31337667, 0.37867500, 0.43137500, 0.51226000, 0.49565333, 0.57213333, 0.62284000,
    0.61870000, 0.68076333, 0.72101667, 0.77838333, 0.83363214, 1.01856667, 1.13127222, 1.10371190,
    1.06819750, 0.95196667, 0.84235000, 0.63558750, 0.56708036, 0.45564167, 0.35380000, 0.32416333,
    0.34918333, 0.42268750, 0.38067500, 0.41432000, 0.41311250, 0.34820333, 0.21971333, 0.08587500,
    -0.06705000, -0.17349250, -0.22847083, -0.29958750, -0.26210000, -0.23115417, -0.08996250, -0.16250000,
    -0.19185000, -0.19186667, -0.40121667, -0.65203810, -0.86277917, -0.91800556, -1.08045682, -1.05119333,
    -0.84775625, -0.67602500, -0.45504167, -0.24374583, -0.33458333, -0.19385667, -0.40442500, -0.34101500,
    -0.65150833, -0.52721905, -0.82706667, -0.63936333, -0.50072667, -0.35925000, -0.20286250, -0.14573333,
    -0.04929583, -0.16920000, 0.01096667, -0.21847500, -0.28542083, -0.13997083, -0.12618750, 0.04497083,
    0.08852500, 0.42406667, 0.59941667, 0.50355000, 0.90435000, 0.73119167, 0.65386429, 0.56130333,
    0.34026667, 0.23716000, 0.24654583, 0.26752083, 0.28051250, 0.31482083, 0.40940417, 0.44661000,
    0.44182000, 0.36421333, 0.45588333, 0.41030667, 0.46474667, 0.43261000, 0.40401250, 0.42845000,
    0.42731000, 0.49215667, 0.52073333, 0.47899500, 0.49218333, 0.35707000, 0.25555750, 0.15628000,
    -0.00057500, -0.18025000, -0.29827500, -0.45958333, -0.48197667, -0.36013333, -0.37142667, -0.28734583,
    -0.22261000, -0.18913750, -0.30767500, -0.26695000, -0.37723333, -0.34482000, -0.46116333, -0.47011500,
    -0.39494167, -0.33738333, -0.21035750, -0.20696667, -0.11738333, -0.18243750, -0.27804667, -0.56415833,
    -0.64445208, -0.85499423, -1.00365758, -1.16269792, -0.99753846, -0.46254000, -0.38305000, -0.20575000,
    0.00826667, 0.11313750, 0.18940000, 0.19915000, 0.15866250, -0.02348333, 0.01668750, 0.00631667,
    0.07286667, 0.13949583, 0.30868750, 0.47769333, 0.58059167, 0.57031667, 0.52726667, 0.46964167,
    0.44243667, 0.40136667, 0.43051000, 0.47732917, 0.50983333, 0.57048500, 0.67115000, 0.89985000,
    1.07538500, 1.08224583, 0.92583889, 0.80410227, 0.58541875, 0.56928333, 0.31238750, 0.28823667,
    0.23494250, 0.09625000, 0.21897500, 0.20157000, 0.15630417, 0.04408750, 0.09921250, -0.05673750,
    -0.11805000, -0.19537917, -0.19277500, -0.21979667, -0.21057500, -0.24661000, -0.22687083, -0.16085417,
    -0.03520000, -0.05116250, -0.08481250, -0.19302000, -0.18048750, -0.38479583, -0.49715833, -0.53850000,
    -0.72490714, -0.57440000, -0.47681667, -0.43795000, -0.38473333, -0.21506667, -0.27676500, -0.23339167,
    -0.39685417, -0.29086000, -0.51048333, -0.49772000, -0.48232500, -0.42944583, -0.36745000, -0.18736667,
    -0.11018333, 0.11940000, -0.05833333, 0.11085000, -0.13940000, -0.15863000, -0.35832083, -0.48761667,
    -0.44051667, -0.29156000, -0.22029000, -0.03363750, 0.25881667, 0.62232500, 0.42147500, 0.58411667,
    0.50742667, 0.46827000, 0.30909500, 0.35256250, 0.20114583, 0.28062917, 0.35573750, 0.43492667,
    0.51590000, 0.56165833, 0.65952500, 0.54298333, 0.49112500, 0.37413000, 0.31003000, 0.33855000,
    0.27641000, 0.20200000, 0.30862083, 0.46209167, 0.47569500, 0.50256667, 0.69134583, 0.49069167,
    0.31231000, 0.15594333, 0.01568250, -0.31986333, -0.37792250, -0.50590833, -0.50713333, -0.48778333,
    -0.48262500, -0.64054167, -0.46949667, -0.48974167, -0.53401667, -0.51080667, -0.58788333, -0.56246667,
    -0.52419167, -0.56826667, -0.54063333, -0.45745667, -0.27817917, -0.23413333, -0.15305417, -0.23677500,
    -0.00205000, -0.26815417, -0.43640000, -0.56333333, -0.56126875, -0.66385238, -0.44657500, -0.42529000,
    -0.28951667, -0.09258333, -0.11848750, 0.05683750, 0.27953333, 0.13597500, 0.22022083, 0.11525000,
    0.09248333, 0.17501250, 0.22140000, 0.23220000, 0.37668750, 0.51741000, 0.64823571, 0.69498929,
    0.74164444, 0.73964167, 0.42632619, 0.10907000, 0.08962500, 0.08947500, -0.00860000, 0.03965000,
    0.17192500, 0.38325000, 0.57085833, 0.65916458, 0.82279583, 0.78199167, 0.71428889, 0.61527083,
    0.50923333, 0.29069167, 0.29053333, 0.28953750, 0.25741000, 0.30008667, 0.31928000, 0.31550500,
    0.29850000, 0.06011250, -0.05088750, -0.19179000, -0.32730417, -0.50517500, -0.51241667, -0.46091667,
    -0.36540000, -0.21255833, -0.13620000, -0.01800000, -0.10285000, -0.13232500, -0.16327083, -0.30764333,
    -0.49486667, -0.58857708, -0.70457500, -0.66215000, -0.74509375, -0.59106667, -0.60654500, -0.65596667,
    -0.58635833, -0.58276667, -0.67964167, -0.65982619, -0.64663333, -0.78428333, -0.71479286, -0.58609167,
    -0.52693000, -0.31419333, -0.16989583, -0.03596250, 0.00320417, 0.14527500, 0.19845000, 0.04120833,
    0.19347500, 0.00120250, 0.03547500, -0.02440833, -0.06256667, 0.28194167, 0.13871667, 0.46070417,
    0.48899000, 0.67946667, 0.50431000, 0.57622500, 0.44035333, 0.47970000, 0.36415000, 0.34591000,
    0.48106667, 0.39307500, 0.64680000, 0.63857500, 0.67003889, 0.71327708, 0.53545833, 0.41440833,
    0.22989500, 0.02455500, -0.23174750, -0.13707000, -0.17551667, -0.07865000, 0.06937083, 0.16950833,
    0.34320000, 0.36854667, 0.32700667, 0.18336250, 0.06718750, -0.09028250, -0.25928333, -0.38559333,
    -0.42143333, -0.43466667, -0.39203667, -0.39622917, -0.40505833, -0.37405000, -0.34085833, -0.38377667,
    -0.64618333, -0.53147500, -0.82556458, -0.77435000, -0.78258750, -0.51550833, -0.46758750, -0.22783750,
    -0.21621250, -0.02761667, -0.13826250, -0.14472917, -0.29248500, -0.54013333, -0.45732500, -0.76556190,
    -0.55819286, -0.50031667, -0.36140500, -0.32330000, -0.23377500, -0.27082500, 0.01982917, -0.10744167,
    -0.05571667, 0.00498750, 0.01776667, 0.05757083, 0.11597500, 0.14451667, 0.34882500, 0.43992500,
    0.52437000, 0.71373333, 0.81172778, 0.79668889, 0.81888750, 0.72121042, 0.56468333, 0.45457667,
    0.47772000, 0.46950000, 0.52086500, 0.51492667, 0.72485000, 0.88353636, 0.94997738, 0.93015385,
    0.80122500, 0.79122500, 0.72278333, 0.61168810, 0.47975833, 0.41150833, 0.43441333, 0.39680667,
    0.49886667, 0.52866190, 0.49490833, 0.38139333, 0.18568750, -0.12556250, -0.33910833, -0.59428889,
    -0.83597000, -1.05966667, -0.92792692, -0.63263333, -0.50752500, -0.39521250, -0.32478333, -0.23867083,
    -0.27480250, -0.43183750, -0.45839167, -0.57753667, -0.65655833, -0.60345714, -0.66284167, -0.57175000,
    -0.47053667, -0.48931000, -0.46085000, -0.41277000, -0.53028667, -0.61863333, -0.73618810, -0.87649643,
    -0.88694000, -0.86224286, -0.65292500, -0.54904583, -0.18824583, -0.06155000, 0.07046667, 0.19862917,
    0.12952500, 0.13246250, 0.06601250, -0.02575417, 0.03944167, -0.12519167, 0.06741667, 0.08227917,
    0.05830000, 0.29637083, 0.26912500, 0.44375333, 0.32342500, 0.33613000, 0.43860000, 0.33373333,
    0.38940833, 0.35742667, 0.43958750, 0.47872333, 0.52040333, 0.70988929, 0.74959524, 0.82328056,
    0.80527292, 0.61343571, 0.64802500, 0.49559167, 0.32774000, 0.27830750, 0.27586250, 0.23892000,
    0.31333750, 0.27537500, 0.40478250, 0.32631667, 0.29598000, 0.24052917, 0.13575000, 0.02555250,
    -0.12516667, -0.21163750, -0.39529333, -0.29800000, -0.28019667, -0.30431250, -0.20950000, -0.17480000,
    -0.22079333, -0.24590500, -0.52180833, -0.63135556, -0.94774286, -0.99417800, -1.20424167, -1.14834853,
    -0.77155625, -0.59203667, -0.41109167, -0.25834583, -0.19343750, -0.24649500, -0.46988333, -0.39164286,
    -0.58823333, -0.62810625, -0.66170833, -0.48585833, -0.29869583, -0.25786333, -0.22053333, -0.03355000,
    -0.12758333, 0.02106000, -0.07032500, -0.10186667, -0.09513333, -0.15380000, 0.01247500, 0.05373333,
    0.30187083, 0.49824000, 0.70065833, 0.89990556, 0.88541818, 0.91357333, 0.74226111, 0.64689167,
    0.48848333, 0.42593333, 0.35167667, 0.39864667, 0.37574583, 0.36434667, 0.52553333, 0.52108000,
    0.55958333, 0.55705714, 0.58610833, 0.55604167, 0.49595833, 0.48671667, 0.52361667, 0.47172667,
    0.50949167, 0.46800000, 0.51901000, 0.49184167, 0.48637143, 0.45424167, 0.32654333, 0.14110000,
    -0.03176750, -0.16270750, -0.29103333, -0.36314167, -0.34834000, -0.35533667, -0.24572083, -0.27347917,
    -0.17798750, -0.18352083, -0.25471000, -0.29638750, -0.34466000, -0.39986250, -0.45515000, -0.52396667,
    -0.41554333, -0.42912000, -0.34196667, -0.20905000, -0.19832083, -0.23206667, -0.33929333, -0.52849167,
    -0.71528095, -0.85045000, -1.09278676, -1.12714881, -1.17593214, -0.85197143, -0.66096000, -0.35963333,
    -0.09312917, 0.00120000, 0.00600833, 0.04396667, 0.00236250, -0.13548333, -0.05352500, -0.03837083,
    0.00365000, 0.10584167, 0.29220000, 0.35686250, 0.43715667, 0.39589500, 0.45281000, 0.30611000,
    0.29359583, 0.28475667, 0.23691250, 0.28840417, 0.34293333, 0.42996000, 0.61147667, 0.71066786,
    1.01943929, 1.06992000, 0.94553438, 0.77441111, 0.58633095, 0.35651667, 0.33020000, 0.19334333,
    0.09527500, 0.13433750, 0.07671250, 0.15388333, 0.17225000, 0.11352917, 0.13680000, -0.07523750,
    -0.00105417, -0.12960000, -0.21692500, -0.26593333, -0.29220250, -0.25182667, -0.18499333, -0.21322083,
    -0.11811250, -0.08192083, -0.10425000, -0.13470417, -0.23135000, -0.27841333, -0.41220000, -0.59017143,
    -0.60585000, -0.72585208, -0.45465333, -0.42242333, -0.29502667, -0.23897500, -0.24186250, -0.16965417,
    -0.30699333, -0.28641333, -0.39590000, -0.41362333, -0.48954167, -0.26392000, -0.38912500, -0.14610833,
    -0.07375000, -0.05800000, 0.10105833, 0.02520000, -0.00231250, -0.12768333, -0.24880833, -0.38527500,
    -0.42889500, -0.41198333, -0.14670000, 0.02765833, 0.23853333, 0.49158667, 0.61915833, 0.72182083,
    0.73975625, 0.61283333, 0.54787857, 0.44761000, 0.39015333, 0.42135417, 0.41538000, 0.54451667,
    0.55508333, 0.66631667, 0.81468409, 0.79288333, 0.61490556, 0.54114524, 0.46307500, 0.39612500,
    0.33694000, 0.32011667, 0.45583333, 0.47062000, 0.60687500, 0.53071071, 0.62794524, 0.54705000,
    0.39809583, 0.20490333, 0.03093750, -0.10791667, -0.26393000, -0.44872083, -0.48748333, -0.45468667,
    -0.40432000, -0.48196667, -0.47728333, -0.35899667, -0.50795833, -0.53056000, -0.53570833, -0.56833500,
    -0.58095000, -0.58455000, -0.57285500, -0.52687500, -0.41186667, -0.23658333, -0.26382500, -0.12948333,
    -0.22997500, -0.28430417, -0.39838000, -0.43637500, -0.63787381, -0.70062500, -0.62236667, -0.57529167,
    -0.36840000, -0.38108333, -0.11052083, -0.00581250, 0.08972917, 0.03633333, 0.08690833, 0.09528750,
    0.03440000, 0.05688333, 0.15198750, 0.21696667, 0.40712083, 0.45214000, 0.50461000, 0.64453333,
    0.65196429, 0.53676667, 0.37004333, 0.15250500, -0.02505000, 0.00342500, -0.11841250, -0.08765000
};
