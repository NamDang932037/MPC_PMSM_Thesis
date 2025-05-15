#include "MPC_Trajectory.h"
#include "structs.h"
#include "tim.h"
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"
#include "arm_math.h"
#include "stm32h7xx_hal.h"
#include <math.h>
#include <stdlib.h>

extern volatile uint32_t Time_Capture[10];
extern volatile uint32_t delta_1us;

void CreateSystemMatrices(float32_t* Ae, float32_t* Be, float32_t* Ce) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Ae[i*3 + j] = 0.0f;
        }
        Be[i] = 0.0f;
        if (i < 2) {
            for (int j = 0; j < 3; j++) {
                Ce[i*3 + j] = 0.0f;
            }
        }
    }


    const float32_t dt_div_j = DT_MPC/J;

    Ae[0*3 + 0] = 1.0f - DT_MPC*Bm/J;
    Ae[0*3 + 2] = dt_div_j;
    Ae[1*3 + 0] = DT_MPC;
    Ae[1*3 + 1] = 1.0f;
    Ae[2*3 + 2] = 1.0f;

    Be[0] = dt_div_j;
    Be[2] = 1.0f;

    Ce[0*3 + 0] = 1.0f;
    Ce[1*3 + 1] = 1.0f;
}


void createPredictionMatrices(
    const float32_t* A_data,
    const float32_t* B_data,
    const float32_t* C_data,
    float32_t* F_data,
    float32_t* G_data) {

    const uint16_t Np = NP;
    const uint16_t Nc = NC;
    
    memset(G_data, 0, 2*Np*Nc*sizeof(float32_t));


    float32_t h_data[2*NP*3] = {0};
    float32_t v_data[2*NP*1] = {0};
    float32_t temp_data[2*3] = {0};

    // h(1:2,1:3) = C
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            h_data[i*3 + j] = C_data[i*3 + j];
        }
    }

    // F(1:2,1:3) = C*A 
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            temp_data[i*3 + j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                temp_data[i*3 + j] += C_data[i*3 + k] * A_data[k*3 + j];
            }
        }
    }
        
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            F_data[i*3 + j] = temp_data[i*3 + j];
        }
    }

    for (int i = 2; i <= Np; i++) {
        // F(2*i-1:2*i,1:3) = F(2*(i-1)-1:2*(i-1),1:3) * A
        float32_t* prev_F_block = &F_data[(2*(i-2))*3];
        float32_t* curr_F_block = &F_data[(2*(i-1))*3];
        
        for (int row = 0; row < 2; row++) {
            for (int col = 0; col < 3; col++) {
                float32_t sum = 0.0f;
                for (int k = 0; k < 3; k++) {
                    sum += prev_F_block[row*3 + k] * A_data[k*3 + col];
                }
                curr_F_block[row*3 + col] = sum;
            }
        }

        // h(2*i-1:2*i,1:3) = h(2*(i-1)-1:2*(i-1),1:3) * A
        float32_t* prev_h_block = &h_data[(2*(i-2))*3];
        float32_t* curr_h_block = &h_data[(2*(i-1))*3];
        
        for (int row = 0; row < 2; row++) {
            for (int col = 0; col < 3; col++) {
                float32_t sum = 0.0f;
                for (int k = 0; k < 3; k++) {
                    sum += prev_h_block[row*3 + k] * A_data[k*3 + col];
                }
                curr_h_block[row*3 + col] = sum;
            }
        }
    }

    // v = h*B
    for (int i = 0; i < 2*Np; i++) {
        v_data[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            v_data[i] += h_data[i*3 + j] * B_data[j];
        }
    }

    // G(1:2*Np,1) = v
    for (int row = 0; row < 2*Np; row++) {
        G_data[row*Nc] = v_data[row];
    }

    for (int i = 1; i < Nc; i++) {
        for (int j = 0; j < 2*(Np-i); j++) {
            G_data[(2*i + j)*Nc + i] = v_data[j];
        }
    }
}

void createConstraintMatrices(ControllerStruct *controller, float32_t* M_data, float32_t* N_data) {
	// Fill N values
	for (int i = 0; i < NC; i++) {
		// First half of N: Te_max - Te
		N_data[i] = Te_max - controller->Te;

		// Second half of N: Te_max + Te
		N_data[NC + i] = Te_max + controller->Te;
	}

	// Fill M matrix (discrete matrix with only main diagonal blocks)
	for (int i = 0; i < NC; i++) {
		for (int j = 0; j <= i; j++) {
			// Set M(i,j) = 1 for first Nc rows
			M_data[i * NC + j] = 1.0f;

			// Set M(Nc+i,j) = -1 for second Nc rows
			M_data[(NC + i) * NC + j] = -1.0f;
		}
	}
}

void planStepTrajectory(ControllerStruct *controller, struct TrajectoryPoint* trajectory) {
	float32_t t = controller->t - 5.0f;
	float32_t speed_ref = controller->v_des;
	
//	if (t < 0) t = 0;
	for (uint8_t i = 1; i <= NP; i++) {
		float32_t t_predict = t + i * DT_MPC;
		if (t_predict <= 1.0f) {
			trajectory[i-1].w_ref = speed_ref;
			trajectory[i-1].angle_ref = speed_ref * (t_predict  + NP*DT_MPC);
		}
		else if (t_predict <= 2.0f) {
			trajectory[i-1].w_ref = -speed_ref;
			trajectory[i-1].angle_ref = speed_ref*1.0f - speed_ref*(t_predict + NP*DT_MPC - 1.0f);
		}
		else {
			trajectory[i-1].w_ref = 0.0f;
			trajectory[i-1].angle_ref = 0.0f;
		}
	}
}



void planSCurveTrajectory(ControllerStruct *controller, struct TrajectoryPoint* trajectory) {
    float32_t t = controller->t - 5.0f;
    float32_t angle_max = controller->p_des - controller->p_offset_MPC;
	float32_t w_max = controller->v_des;
	float32_t a_max = 2.0f * 2.0f * w_max * w_max / angle_max;
	float32_t tc = w_max / a_max;
	float32_t j_max = a_max / tc;

	float32_t a_max_limit = 50.0f;

	// Check if acceleration exceeds the limit
	if (a_max > a_max_limit) {
	    a_max = a_max_limit;
	    tc = w_max / a_max;  // Recalculate tc with limited a_max
	    j_max = a_max / tc;  // Recalculate j_max
	}
    if (w_max > sqrtf(0.5f * angle_max * a_max)) {
        w_max = sqrtf(0.5f * angle_max * a_max);
        // Recalculate tc and j_max with new w_max
        tc = w_max / a_max;
        j_max = a_max / tc;
    }

	if (a_max < -a_max_limit) {
	    a_max = -a_max_limit;
	    tc = w_max / a_max;  // Recalculate tc with limited a_max
	    j_max = a_max / tc;  // Recalculate j_max
	}
    if (w_max < -sqrtf(0.5f * angle_max * a_max)) {
        w_max = -sqrtf(0.5f * angle_max * a_max);
        // Recalculate tc and j_max with new w_max
        tc = w_max / a_max;
        j_max = a_max / tc;
    }


    // Calculate trajectory parameters
    float32_t qi = j_max * tc * tc * tc;
    float32_t tm = (angle_max - 2.0f * qi) / w_max;
    float32_t tf = 4.0f * tc + tm;

    for (int i = 1; i <= NP; i++) {
        float32_t t_predict = t + i * DT_MPC;
        float32_t t_with_offset = t_predict + NP * DT_MPC;

        if (t_predict < tc) {
            trajectory[i-1].w_ref = 0.5f * j_max * t_with_offset * t_with_offset;
            trajectory[i-1].angle_ref = (1.0f/6.0f) * j_max * t_with_offset * t_with_offset * t_with_offset;
        }
        else if (t_predict < 2.0f * tc) {
            float32_t dt = t_with_offset - tc;
            trajectory[i-1].w_ref = 0.5f * j_max * tc * tc + j_max * tc * dt - 0.5f * j_max * dt * dt;
            trajectory[i-1].angle_ref = (1.0f/6.0f) * j_max * tc * tc * tc +
                                     0.5f * j_max * tc * tc * dt +
                                     0.5f * j_max * tc * dt * dt -
                                     (1.0f / 6.0f) * j_max * dt * dt * dt;
        }
        else if (t_predict < 2.0f * tc + tm) {
            trajectory[i-1].w_ref = w_max;
            trajectory[i-1].angle_ref = j_max * tc * tc * tc + w_max * (t_with_offset - 2.0f * tc);
        }
        else if (t_predict < 3.0f * tc + tm) {
            float32_t dt = t_with_offset - 2.0f * tc - tm;
            trajectory[i-1].w_ref = w_max - 0.5f * j_max * dt * dt;
            trajectory[i-1].angle_ref = j_max * tc * tc * tc +
                                     w_max * tm +
                                     w_max * dt -
                                     (1.0f/6.0f) * j_max * dt * dt * dt;
        }
        else if (t_predict < tf) {
            float32_t dt = t_with_offset - 3.0f * tc - tm;
            trajectory[i-1].w_ref = 0.5f * j_max * tc * tc - a_max * dt + 0.5f * j_max * dt * dt;
            trajectory[i-1].angle_ref = angle_max -
                                     (1.0f/6.0f) * j_max *
                                     (tf - t_with_offset) *
                                     (tf - t_with_offset) *
                                     (tf - t_with_offset);
        }
        else {
            trajectory[i-1].w_ref = 0;
            trajectory[i-1].angle_ref = angle_max;
        }
    }
}


void planSineTrajectory(ControllerStruct *controller, struct TrajectoryPoint* trajectory) {
	float32_t t = controller->t - 5.0f;
	float32_t speed_ref = controller->v_des;
	
	if (t < 0) t = 0;

	for (int i = 1; i <= NP; i++) {
		float32_t t_predict = t + i * DT_MPC;
		trajectory[i-1].w_ref = speed_ref * cos(M_PI * t_predict);
		trajectory[i-1].angle_ref = speed_ref / M_PI * sin(M_PI * t_predict) + controller->p_offset_MPC;
	}
}

void planLSPBTrajectory(ControllerStruct *controller, struct TrajectoryPoint* trajectory) {
	float32_t t = controller->t - 5.0f;
	if (t < 0) t = 0;
	float32_t angle_max = controller->p_des - controller->p_offset_MPC;
	float32_t w_max = controller->v_des;
	float32_t a_max = 2.0f * 2.0f * w_max * w_max / angle_max;
	float32_t tc = w_max / a_max;

	float32_t a_max_limit = 50.0f;

	if (a_max > a_max_limit) {
		a_max = a_max_limit;
		tc = w_max / a_max;
	}
	if (w_max > sqrtf(0.5f * angle_max * a_max)) {
		w_max = sqrtf(0.5f * angle_max * a_max);
		tc = w_max / a_max;
	}

	if (a_max < -a_max_limit) {
		a_max = -a_max_limit;
		tc = w_max / a_max;
	}
	if (w_max < -sqrtf(0.5f * angle_max * a_max)) {
		w_max = -sqrtf(0.5f * angle_max * a_max);
		tc = w_max / a_max;
	}

	// LSPB parameters
	float32_t qc = 0.5f * a_max * tc *tc;
	float32_t tm = (angle_max - 2*qc)/w_max;
	float32_t tf = 2*tc + tm;


	for (int i = 1; i <= NP; i++) {
		float32_t t_predict = t + i * DT_MPC;

		if (t_predict <= tc) {
			// Initial acceleration
			trajectory[i-1].w_ref = a_max*(t_predict + NP*DT_MPC);
			trajectory[i-1].angle_ref = 0.5f*a_max*(t_predict - t)*(t_predict - t);
		}
		else if (t_predict <= tc + tm) {
			// Constant velocity
			trajectory[i-1].w_ref = w_max;
			trajectory[i-1].angle_ref = qc + w_max*(t_predict + NP*DT_MPC - tc);
		}
		else if (t_predict <= tf) {
			// Final deceleration
			trajectory[i-1].w_ref = w_max - a_max*(t_predict + NP*DT_MPC - (tc + tm));
			trajectory[i-1].angle_ref = qc + w_max*tm + w_max*(t_predict + NP*DT_MPC - (tc + tm))
							  - 0.5f*a_max*(t_predict + NP*DT_MPC - (tc + tm))*(t_predict + NP*DT_MPC - (tc + tm));
		}
		else {
			// After motion completion
			trajectory[i-1].w_ref = 0;
			trajectory[i-1].angle_ref = angle_max;
		}
	}
}

void calculateQPMatrices(
	const float32_t* G_data,
	const float32_t* Qy_data,
	const float32_t* R_u_data,
	const float32_t* F_data,
	const float32_t* X_data,
	const float32_t* Rs_data,
	float32_t* H_data,
	float32_t* f_data) {


    static float32_t FX_data[2 * NP];
    static float32_t G_trans_data[NC * 2 * NP];  // Ma trận G chuyển vị

    const int Nc = NC;
    const int Np = NP;
    const int rows_G = 2 * Np;
    const int cols_G = Nc;

    for (int i = 0; i < cols_G; i++) {
        for (int j = 0; j < rows_G; j++) {
            G_trans_data[i * rows_G + j] = G_data[j * cols_G + i];
        }
    }

    
    // H = G^T * Qy * G + R_u
    for (int i = 0; i < cols_G; i++) {
        for (int j = i; j < cols_G; j++) {  
            float32_t sum = 0.0f;
            
            for (int k = 0; k < Np; k++) {
                float32_t qy_k0 = Qy_data[(2*k + 0) * (2*NP) + (2*k + 0)]; 
                float32_t qy_k1 = Qy_data[(2*k + 1) * (2*NP) + (2*k + 1)]; 
                
                float32_t g_ik0 = G_trans_data[i * rows_G + 2*k];
                float32_t g_ik1 = G_trans_data[i * rows_G + 2*k + 1];
                float32_t g_jk0 = G_data[2*k * cols_G + j];
                float32_t g_jk1 = G_data[(2*k + 1) * cols_G + j];
                
                sum += g_ik0 * qy_k0 * g_jk0 + g_ik1 * qy_k1 * g_jk1;
            }
            
            float32_t h_value = sum + R_u_data[i * cols_G + j];
            H_data[i * cols_G + j] = h_value;
            if (i != j) {
                H_data[j * cols_G + i] = h_value;
            }
        }
    }

    for (int i = 0; i < rows_G; i++) {
        // Unrolling the inner loop for better instruction pipelining
        float32_t sum = 0.0f;
        const float32_t* F_row = &F_data[i * 3]; 

        sum += F_row[0] * X_data[0];
        sum += F_row[1] * X_data[1];
        sum += F_row[2] * X_data[2];
        
        FX_data[i] = sum;
    }

    for (int i = 0; i < cols_G; i++) {
        float32_t sum = 0.0f;
        
        for (int k = 0; k < Np; k++) {

            float32_t qy_k0 = Qy_data[(2*k + 0) * (2*NP) + (2*k + 0)]; 
            float32_t qy_k1 = Qy_data[(2*k + 1) * (2*NP) + (2*k + 1)]; 
            
            // G^T[i,k] * Qy[k,k] * (FX[k] - Rs[k])
            float32_t diff0 = FX_data[2*k] - Rs_data[2*k];
            float32_t diff1 = FX_data[2*k + 1] - Rs_data[2*k + 1];
            
            sum += G_trans_data[i * rows_G + 2*k] * qy_k0 * diff0 +
                   G_trans_data[i * rows_G + 2*k + 1] * qy_k1 * diff1;
        }
        
        f_data[i] = sum;
    }
}

void createReferenceVector(ControllerStruct *controller, float32_t* Rs) {
	memset(Rs, 0, 2 * NP * sizeof(float32_t));

	int plan_mode = controller->plan_mode;
	
    struct TrajectoryPoint trajectory[NP];

    switch(plan_mode) {
        case 0: // Step mode
            planStepTrajectory(controller, trajectory);
            break;
        case 1: // S-curve mode
            planSCurveTrajectory(controller, trajectory);
            break;
        case 2: // Sine mode
            planSineTrajectory(controller, trajectory);
            break;
        case 3: // LSPB mode
            planLSPBTrajectory(controller, trajectory);
            break;
        default:
            return;
    }


    for (int i = 0; i < NP; i++) {
        Rs[2 * i] = trajectory[i].w_ref;
        Rs[2 * i + 1] = trajectory[i].angle_ref;
    }
	

	controller->v_rs = Rs[0];
	controller->p_rs = Rs[1];
}


void solveHildreth(const float32_t* H_data, const float32_t* f_data,
                   const float32_t* M_data, const float32_t* N_data,
                   float32_t* delta_U) {
    int n = 2 * NC;  
    float NumofLoops = 0;

    static float32_t H_inv_data[NC * NC];
    static float32_t MT_data[NC * 2 * NC];  
    static float32_t M_H_inv_data[2 * NC * NC];  // M * H_inv
    static float32_t P_data[2 * NC * 2 * NC];  // M * H_inv * M^T
    static float32_t M_H_inv_f_data[2 * NC];  // M * H_inv * f
    static float32_t d_data[2 * NC];  // N + M * H_inv * f
    static float32_t lambda[2 * NC];  // Vector lambda
    static float32_t lambda_prev[2 * NC];  // Vector lambda previous
    static float32_t M_T_lambda_data[NC];  // M^T * lambda
    static float32_t f_M_lambda_data[NC];  // f + M^T * lambda

    memset(H_inv_data, 0, NC * NC * sizeof(float32_t));
    memset(MT_data, 0, NC * 2 * NC * sizeof(float32_t));
    memset(M_H_inv_data, 0, 2 * NC * NC * sizeof(float32_t));
    memset(P_data, 0, 2 * NC * 2 * NC * sizeof(float32_t));
    memset(M_H_inv_f_data, 0, 2 * NC * sizeof(float32_t));
    memset(d_data, 0, 2 * NC * sizeof(float32_t));
    memset(lambda, 0, 2 * NC * sizeof(float32_t));
    memset(lambda_prev, 0, 2 * NC * sizeof(float32_t));
    memset(M_T_lambda_data, 0, NC * sizeof(float32_t));
    memset(f_M_lambda_data, 0, NC * sizeof(float32_t));


    float32_t identity[NC * NC] = {0};
    for (int i = 0; i < NC; i++) {
        identity[i * NC + i] = 1.0f;
    }

    float32_t H_temp[NC * NC];
    for (int i = 0; i < NC * NC; i++) {
        H_temp[i] = H_data[i];
    }

    // Gauss-Jordan
    for (int i = 0; i < NC; i++) {
        float32_t pivot = H_temp[i * NC + i];
        for (int j = 0; j < NC; j++) {
            H_temp[i * NC + j] /= pivot;
            identity[i * NC + j] /= pivot;
        }

        for (int k = 0; k < NC; k++) {
            if (k != i) {
                float32_t factor = H_temp[k * NC + i];
                for (int j = 0; j < NC; j++) {
                    H_temp[k * NC + j] -= factor * H_temp[i * NC + j];
                    identity[k * NC + j] -= factor * identity[i * NC + j];
                }
            }
        }
    }


    for (int i = 0; i < NC * NC; i++) {
        H_inv_data[i] = identity[i];
    }

    // delta_U = -H_inv * f
    for (int i = 0; i < NC; i++) {
        delta_U[i] = 0.0f;
        for (int j = 0; j < NC; j++) {
            delta_U[i] += H_inv_data[i * NC + j] * f_data[j];
        }
        delta_U[i] = -delta_U[i];  
    }

    // M * delta_U <= N ?
    static float32_t M_delta_U_data[2 * NC];
    for (int i = 0; i < n; i++) {
        M_delta_U_data[i] = 0.0f;
        for (int j = 0; j < NC; j++) {
            M_delta_U_data[i] += M_data[i * NC + j] * delta_U[j];
        }
    }

    bool constraints_violated = false;
    for (int i = 0; i < n; ++i) {
        if (M_delta_U_data[i] > N_data[i]) {
            constraints_violated = true;
            break;
        }
    }

    if (constraints_violated) {
        // MT = M^T
        for (int i = 0; i < NC; i++) {
            for (int j = 0; j < n; j++) {
                MT_data[i * n + j] = M_data[j * NC + i];
            }
        }

        // M_H_inv = M * H_inv
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < NC; j++) {
                M_H_inv_data[i * NC + j] = 0.0f;
                for (int k = 0; k < NC; k++) {
                    M_H_inv_data[i * NC + j] += M_data[i * NC + k] * H_inv_data[k * NC + j];
                }
            }
        }

        // P = M * H_inv * M^T
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                P_data[i * n + j] = 0.0f;
                for (int k = 0; k < NC; k++) {
                    P_data[i * n + j] += M_H_inv_data[i * NC + k] * MT_data[k * n + j];
                }
            }
        }

        // d = N + M * H_inv * f
        for (int i = 0; i < n; i++) {
            M_H_inv_f_data[i] = 0.0f;
            for (int j = 0; j < NC; j++) {
                M_H_inv_f_data[i] += M_H_inv_data[i * NC + j] * f_data[j];
            }
            d_data[i] = N_data[i] + M_H_inv_f_data[i];
        }

        float32_t P_diag[2 * NC];
        for (int i = 0; i < n; i++) {
            P_diag[i] = P_data[i * n + i];
        }

        // Hildreth
        float alpha = 1.0;
        float epsilon = 100e-3;
        int maxIter = 20;

        while (alpha > epsilon) {
            if (NumofLoops >= maxIter) break;

            for (int i = 0; i < n; i++) {
                lambda_prev[i] = lambda[i];
            }

            for (int i = 0; i < n; ++i) {
                float w = d_data[i];

                // P(i,:)*lambda
                const float32_t* P_row = &P_data[i * n];

                int j = 0;
                for (; j < n - 7; j += 8) {
                    w += P_row[j] * lambda[j] +
                         P_row[j+1] * lambda[j+1] +
                         P_row[j+2] * lambda[j+2] +
                         P_row[j+3] * lambda[j+3] +
                         P_row[j+4] * lambda[j+4] +
                         P_row[j+5] * lambda[j+5] +
                         P_row[j+6] * lambda[j+6] +
                         P_row[j+7] * lambda[j+7];
                }

                for (; j < n; j++) {
                    w += P_row[j] * lambda[j];
                }

                w -= P_diag[i] * lambda[i];

                // lambda(i) = max(0, -w/P(i,i))
                lambda[i] = fmaxf(0.0f, -w / P_diag[i]);
            }

            // alpha = (lambda-lambda_prev)'*(lambda-lambda_prev)
            alpha = 0.0f;
            for (int i = 0; i < n; ++i) {
                float32_t diff = lambda[i] - lambda_prev[i];
                alpha += diff * diff;
            }
            NumofLoops += 1;
        }

        controller.LoopMPC = NumofLoops;

        // M^T * lambda
        for (int i = 0; i < NC; i++) {
            M_T_lambda_data[i] = 0.0f;
            for (int j = 0; j < n; j++) {
                M_T_lambda_data[i] += MT_data[i * n + j] * lambda[j];
            }
        }

        // f + M^T * lambda
        for (int i = 0; i < NC; i++) {
            f_M_lambda_data[i] = f_data[i] + M_T_lambda_data[i];
        }

        // delta_U = -H_inv * (f + M^T * lambda)
        for (int i = 0; i < NC; i++) {
            delta_U[i] = 0.0f;
            for (int j = 0; j < NC; j++) {
                delta_U[i] += H_inv_data[i * NC + j] * f_M_lambda_data[j];
            }
            delta_U[i] = -delta_U[i];  
        }
    }
}



//Main fucntion
void MPC_Calculate(ControllerStruct *controller) {
    static float32_t A_data[3 * 3];
    static float32_t B_data[3 * 1];
    static float32_t C_data[2 * 3];
    static float32_t F_data[2 * NP * 3];
    static float32_t G_data[2 * NP * NC];
    static float32_t Qy_data[2 * NP * 2 * NP];
    static float32_t R_u_data[NC * NC];
    static float32_t M_data[2 * NC * NC];
    static float32_t N_data[2 * NC];
    static float32_t Rs_data[2 * NP];
    static float32_t X_data[3];
    static float32_t H_data[NC * NC];
    static float32_t f_data[NC];
    static float32_t delta_U[NC];
    

    static bool C_initialized = false;
    if (!C_initialized) {
        memset(C_data, 0, 2*3*sizeof(float32_t));
        C_data[0*3 + 0] = 1.0f;
        C_data[1*3 + 1] = 1.0f;
        C_initialized = true;
    }
    

    static bool Qy_initialized = false;
    if (!Qy_initialized) {
        memset(Qy_data, 0, 2 * NP * 2 * NP * sizeof(float32_t));
        for(int i = 0; i < NP; i++) {
            Qy_data[(2*i + 0) * (2*NP) + (2*i + 0)] = 15.0f;     // Q(0,0)
            Qy_data[(2*i + 1) * (2*NP) + (2*i + 1)] = 65.0f;     // Q(1,1)
        	//Calib cogging usage
//            Qy_data[(2*i + 0) * (2*NP) + (2*i + 0)] = 20.0f;     // Q(0,0)
//            Qy_data[(2*i + 1) * (2*NP) + (2*i + 1)] = 10.0f;     // Q(1,1)
        }
        Qy_initialized = true;
    }
    
    static bool R_initialized = false;
    if (!R_initialized) {
        memset(R_u_data, 0, NC * NC * sizeof(float32_t));
        for(int i = 0; i < NC; i++) {
            R_u_data[(i + 0) * (NC) + (i + 0)] = 0.001f;   // R_v(0,0)
        }
        R_initialized = true;
    }
    
    float32_t t = controller->t - 5.0f;
    
    // Step 0: 
    if(controller->t < 0.0f) {
        controller->i_d_des = 0;
        controller->i_q_des = 0;
        controller->Te_des = 0;
        controller->p_des = 0;
        controller->v_des = 0;
        return;
    }

    if (t < 0) t = 0;
    

    if (controller->plan_mode == 1) {
        float32_t angle_max = controller->p_des - controller->p_offset_MPC;
        float32_t w_max = controller->v_des;
        float32_t a_max = 2.0f * 2.0f * w_max * w_max / angle_max;
        float32_t tc = w_max / a_max;
        float32_t j_max = a_max / tc;
        float32_t a_max_limit = 50.0f;

        if (fabsf(a_max) > a_max_limit) {
            a_max = copysignf(a_max_limit, a_max);
            tc = w_max / a_max;
            j_max = a_max / tc;
        }
        
        if (fabsf(w_max) > sqrtf(0.5f * fabsf(angle_max * a_max))) {
            w_max = copysignf(sqrtf(0.5f * fabsf(angle_max * a_max)), w_max);
            tc = w_max / a_max;
            j_max = a_max / tc;
        }

        float32_t qi = j_max * tc * tc * tc;
        float32_t tm = (angle_max - 2.0f * qi) / w_max;
        float32_t tf = 4.0f * tc + tm;
        const float32_t one_sixth = 1.0f/6.0f;

        if (t < 0.0f) {
            controller->v_plan = 0.0f;
            controller->p_plan = 0.0f + controller->p_offset_MPC;
        }
        else if (t < tc) {
            float32_t t2 = t * t;
            controller->v_plan = 0.5f * j_max * t2;
            controller->p_plan = one_sixth * j_max * t2 * t + controller->p_offset_MPC;
        }
        else if (t < 2.0f * tc) {
            float32_t dt = t - tc;
            float32_t tc2 = tc * tc;
            float32_t dt2 = dt * dt;
            
            controller->v_plan = 0.5f * j_max * tc2 + j_max * tc * dt - 0.5f * j_max * dt2;
            controller->p_plan = one_sixth * j_max * tc2 * tc +
                                 0.5f * j_max * tc2 * dt +
                                 0.5f * j_max * tc * dt2 -
                                 one_sixth * j_max * dt2 * dt
                                 + controller->p_offset_MPC;
        }
        else if (t < 2.0f * tc + tm) {
            controller->v_plan = w_max;
            controller->p_plan = j_max * tc * tc * tc + w_max * (t - 2.0f * tc)
                                 + controller->p_offset_MPC;
        }
        else if (t < 3.0f * tc + tm) {
            float32_t dt = t - (2.0f * tc + tm);
            float32_t dt2 = dt * dt;
            
            controller->v_plan = w_max - 0.5f * j_max * dt2;
            controller->p_plan = j_max * tc * tc * tc + w_max * tm +
                                 w_max * dt - one_sixth * j_max * dt2 * dt
                                 + controller->p_offset_MPC;
        }
        else if (t < tf) {
            float32_t dt = t - (3.0f * tc + tm);
            float32_t tc2 = tc * tc;
            float32_t dt2 = dt * dt;
            float32_t tf_t = tf - t;
            float32_t tf_t3 = tf_t * tf_t * tf_t;
            
            controller->v_plan = 0.5f * j_max * tc2 - a_max * dt + 0.5f * j_max * dt2;
            controller->p_plan = angle_max - one_sixth * j_max * tf_t3
                                + controller->p_offset_MPC;
        }
        else {
            controller->v_plan = 0.0f;
            controller->p_plan = angle_max + controller->p_offset_MPC;
        }
    } else {
        switch(controller->plan_mode) {
            case 0: { // Step mode
                if (t <= 1.0f) {
                    controller->v_plan = controller->v_des;
                    controller->p_plan = controller->v_des * t + controller->p_offset_MPC;
                } else if (t <= 2.0f) {
                    controller->v_plan = -controller->v_des;
                    controller->p_plan = controller->v_des * 1.0f - controller->v_des * (t - 1.0f) + controller->p_offset_MPC;
                } else {
                    controller->v_plan = 0;
                    controller->p_plan = 0 + controller->p_offset_MPC;
                }
                break;
            }
            case 2: { // Sine mode
                controller->v_plan = controller->v_des * cos(2*M_PI * t);
                controller->p_plan = (controller->v_des / (2*M_PI)) * sin(2*M_PI * t) + controller->p_offset_MPC;
                break;
            }
            case 3: { // LSPB mode

                if (t < 0) t = 0;

                	float32_t angle_max = controller->p_des - controller->p_offset_MPC;
                	float32_t w_max = controller->v_des;
                	float32_t a_max = 2.0f * 2.0f * w_max * w_max / angle_max;
                	float32_t tc = w_max / a_max;

                	float32_t a_max_limit = 50.0f;
                	// Giới hạn gia tốc và vận tốc
                	if (a_max > a_max_limit) {
                		a_max = a_max_limit;
                		tc = w_max / a_max;
                	}
                	if (w_max > sqrtf(0.5f * angle_max * a_max)) {
                		w_max = sqrtf(0.5f * angle_max * a_max);
                		tc = w_max / a_max;
                	}

                	if (a_max < -a_max_limit) {
                		a_max = -a_max_limit;
                		tc = w_max / a_max;
                	}
                	if (w_max < -sqrtf(0.5f * angle_max * a_max)) {
                		w_max = -sqrtf(0.5f * angle_max * a_max);
                		tc = w_max / a_max;
                	}

                	// LSPB parameters
                	float32_t qc = 0.5f * a_max * tc *tc;
                	float32_t tm = (angle_max - 2*qc)/w_max;
                	float32_t tf = 2*tc + tm;

					if (t < tc) {
						// Initial acceleration
						controller->v_plan = w_max/tc*t;
						controller->p_plan = w_max/(2*tc)*t*t + controller->p_offset_MPC;
					}
					else if (t < tc + tm) {
						// Constant velocity
						controller->v_plan = w_max;
						controller->p_plan = (w_max/2)*tc + w_max*(t - tc) + controller->p_offset_MPC;
					}
					else if (t < tf) {
						// Final deceleration
						controller->v_plan = w_max*(tf - t)/(tf - (tc + tm));
						controller->p_plan = w_max/2*tc + w_max*tm + w_max*(t - tc - tm) - (w_max/(2*(tf - tc - tm)))*(t - tc - tm)*(t - tc - tm)
										  + controller->p_offset_MPC;
					}
					else {
						// After motion completion
						controller->v_plan = 0;
						controller->p_plan = angle_max + controller->p_offset_MPC;
					}

                break;
            }
        }
    }
    



    Time_Capture[0] = TIM17->CNT;
    // Step 1
    CreateSystemMatrices(A_data, B_data, C_data);
    Time_Capture[1] = TIM17->CNT;

    // Step 2
    createPredictionMatrices(A_data, B_data, C_data, F_data, G_data);
    Time_Capture[2] = TIM17->CNT;

    // Step 3
    createConstraintMatrices(controller, M_data, N_data);
    Time_Capture[3] = TIM17->CNT;

    // Step 4
    createReferenceVector(controller, Rs_data);
    Time_Capture[4] = TIM17->CNT;

    // Step 5
    X_data[0] = controller->dtheta_mech;
    X_data[1] = controller->theta_mech;
    X_data[2] = controller->Te - controller->Tm_Filtered;
//    X_data[2] = controller->Te;
    Time_Capture[5] = TIM17->CNT;

    // Step 6
    calculateQPMatrices(G_data, Qy_data, R_u_data, F_data, X_data, Rs_data, H_data, f_data);
    Time_Capture[6] = TIM17->CNT;

    // Step 7
    solveHildreth(H_data, f_data, M_data, N_data, delta_U);
    Time_Capture[7] = TIM17->CNT;
    controller->Te_des += delta_U[0];
    controller->Te_des = fmin(fmax(controller->Te_des, -Te_max), Te_max);
    controller->i_q_des = controller->Te_des/TORQUE_CONSTANT;
    controller->i_d_des = 0;
    Time_Capture[8] = TIM17->CNT;

    return;
}


