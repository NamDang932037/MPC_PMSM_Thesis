#ifndef INC_FOC_H_
#define INC_FOC_H_

#include <stdint.h>
#include "position_sensor.h"

#define SQRT3       1.7320508075688772f
#define SQRT3_DIV_2 0.8660254037844386f
#define ONE_DIV_3   0.3333333333333333f
#define TWO_DIV_3   0.6666666666666666f
#define THREE_DIV_PI  0.9549296585513721f   // 3/pi

// Định nghĩa struct TrajectoryPoint độc lập
typedef struct TrajectoryPoint {
	float w_ref;
	float angle_ref;
} TrajectoryPoint;

typedef struct{
	uint32_t tim_ch_w;										// Terminal W timer channel
    int adc_a_raw, adc_b_raw, adc_c_raw, adc_vbus_raw;      // Raw ADC Values
    float i_a, i_b, i_c;                                    // Phase currents
    float v_bus, v_bus_filt;                                // DC link voltage
    float theta_mech, theta_elec;                           // Rotor mechanical and electrical angle
    float dtheta_mech, dtheta_elec, dtheta_elec_filt;       // Rotor mechanical and electrical angular velocity
    float i_d, i_q, i_q_filt, i_d_filt;                     // D/Q currents
    float i_mag, i_mag_max;									// Current magnitude
    float v_d, v_q;                                         // D/Q voltages
    float dtc_u, dtc_v, dtc_w;                              // Terminal duty cycles
    float v_u, v_v, v_w;                                    // Terminal voltages
    float i_scale, k_d, k_q, ki_d, ki_q, ki_fw, alpha;      // Current loop gains, current reference filter coefficient
    float flux_linkage;
    float d_int, q_int;                                     // Current error integrals
    float d_err, q_err;
    int adc_a_offset, adc_b_offset, adc_c_offset, adc_vbus_offset; 		// ADC offsets
    float Te_des,i_d_des, i_q_des, i_d_des_filt, i_q_des_filt, t_ff_filt;     // Current references
    float pos_err, pos_int;
    float max_current_integral;							// Anti-windup for PI current controller
    float Te,Pre_Te;										// Electromanetic Torque
    float rate_hz;											// PI controller frequency
    float Tm, Tm_Filtered;												// External Load Torque
    int plan_mode;
    int loop_count;                                         // Degubbing counter
    int timeout;                                            // Watchdog counter
//    int mode;
    int ovp_flag;                                           // Over-voltage flag
    int oc_flag;											// Over-current flag
    int phase_order;
    float t;												// Time
    float p_plan, v_plan, p_rs, v_rs;
    TrajectoryPoint trajectory_point;
    //KF
    float pos_estimate, vel_estimate;
    uint8_t enable_kf, kf_flag, kf_done, kf_firsttime;
    union{
    	float commands[6];									// Making this easier to pass around without including foc.h everywhere
    	struct{
    		float p_des, v_des, kp, kd, t_ff, mode;               // Desired position, velocity, gains, torque
    	};
    };

    float ki;
    float v_max, v_ref, fw_int, v_margin;                   // output voltage magnitude, field-weakening integral
    int otw_flag;                                           // Over-temp warning
    float i_max;											// Maximum current
    float inverter_tab[128];								// Inverter linearization table
    uint8_t invert_dtc;										// Inverter duty cycle inverting/non-inverting
    uint8_t Control_Ready;
    uint8_t Valid_Control_Cmd;
    uint8_t fault;

    uint8_t LoopMPC;
    float p_offset_MPC;
    float t_MPC_Offset;

    float tracking_pos_error,d_tracking_pos_error,offset_v;

} ControllerStruct;

typedef struct{
    double temperature;                                     // Estimated temperature
    float temp_measured;									// "Measured" temperature computed from resistance
    float qd_in, qd_out;									// Thermal power in and out
    float resistance;										// Motor resistance
    float k;												// Temperature observer gain
    float trust;											// Temperature observer "trust' (kind of like 1/covariance)
    float delta_t;											// Temperature rise
    }   ObserverStruct;


#ifdef __cplusplus
extern "C" {
#endif

void set_dtc(ControllerStruct *controller);
void analog_sample(ControllerStruct *controller);
void abc(float theta, float d, float q, float *a, float *b, float *c);
void dq0(float theta, float a, float b, float c, float *d, float *q);
void svm(float v_max, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w);
void zero_current(ControllerStruct *controller);
void reset_foc(ControllerStruct *controller);
void reset_observer(ObserverStruct *observer);
void init_controller_params(ControllerStruct *controller);
void commutate(ControllerStruct *controller, EncoderStruct *encoder);
void torque_control(ControllerStruct *controller);
void limit_current_ref (ControllerStruct *controller);
void update_observer(ControllerStruct *controller, ObserverStruct *observer);
void field_weaken(ControllerStruct *controller);
float linearize_dtc(ControllerStruct *controller, float dtc);
void zero_commands(ControllerStruct * controller);
void ExponentialFilter(ControllerStruct * controller, float alpha);
void offset_current(ControllerStruct *controller);



void SVPWM_v2(ControllerStruct* controller, EncoderStruct* encoder);
void Cal_PWM_Val(ControllerStruct* controller, float a, float b, float c, int shift);
float Scale_fdx(float fdx, float fd_other);
float Cal_Scale(float val);

void PositionControl(ControllerStruct *controller, EncoderStruct* encoder);
void FOC(ControllerStruct *controller, EncoderStruct *encoder);
float PID_Torque(ControllerStruct* controller, EncoderStruct* encoder);
float Torque_to_Current(float torque_in);


#ifdef __cplusplus
}
#endif

#endif /* INC_FOC_H_ */
