#include "KF.h"

void KF(ControllerStruct *controller, EncoderStruct *encoder) {

    static float xk_1k_1[3] = {0.0f, 0.0f, 0.0f};
    static float Pk_1k_1[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    static float uk_1 = 0.0f;

	if(controller->kf_firsttime) {
		//Initial Value
		xk_1k_1[0] = encoder->filtered_vel;
        xk_1k_1[1] = encoder->angle_multiturn[0];
        xk_1k_1[2] = 0.0f;
        uk_1 = controller->Te;
        controller->p_offset_MPC = encoder->angle_multiturn[0];
//        controller->p_offset_MPC = 0;
        controller->d_tracking_pos_error = 0;
        controller->tracking_pos_error = 0;
        controller->offset_v = 0;
        controller->kf_firsttime = 0;
	}

    float A[3][3] = {
        {-Bm/J, 0, -1/J},
        {1, 0, 0},
        {0, 0, 0}
    };
    float B[3] = {1/J, 0, 0};
    float C[2][3] = {
        {1, 0, 0},
        {0, 1, 0}
    };


    float Ak_1[3][3], Bk[3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Ak_1[i][j] = (i == j ? 1.0f : 0.0f) + DT_KF * A[i][j];
        }
        Bk[i] = DT_KF * B[i];
    }


    float Q[3][3] = {{0.01f, 0, 0}, {0, 0.01f, 0}, {0, 0, 50.0f}};
    float R[2][2] = {{10.0f, 0}, {0, 10.0f}};  

    // Time update 
    float xk_k_1[3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            xk_k_1[i] += Ak_1[i][j] * xk_1k_1[j];
        }
        xk_k_1[i] += Bk[i] * uk_1;
    }

    // Pk_k_1 = Ak_1 * Pk_1k_1 * Ak_1' + Q
    float Pk_k_1[3][3] = {0};
    float temp[3][3] = {0};

    // temp = Ak_1 * Pk_1k_1
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            temp[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                temp[i][j] += Ak_1[i][k] * Pk_1k_1[k][j];
            }
        }
    }

    // Pk_k_1 = temp * Ak_1' + Q
     for (int i = 0; i < 3; i++) {
         for (int j = 0; j < 3; j++) {
             Pk_k_1[i][j] = 0;
             for (int k = 0; k < 3; k++) {
                 Pk_k_1[i][j] += temp[i][k] * Ak_1[j][k];  
             }
             Pk_k_1[i][j] += Q[i][j];
         }
     }

    // Measure update 
    float yk[2] = {encoder->filtered_vel, encoder->angle_multiturn[0]};
    float ykk_1[2] = {0}; 

    // ykk_1 = C * xk_k_1
    for (int i = 0; i < 2; i++) {
        ykk_1[i] = 0;
        for (int j = 0; j < 3; j++) {
            ykk_1[i] += C[i][j] * xk_k_1[j];
        }
    }

    // Pxy = Pk_k_1 * C'
    float Pxy[3][2] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            Pxy[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                Pxy[i][j] += Pk_k_1[i][k] * C[j][k]; 
            }
        }
    }

    // Pyy = C * Pk_k_1 * C' + R
    float Pyy[2][2] = {0};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            Pyy[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                for (int l = 0; l < 3; l++) {
                    Pyy[i][j] += C[i][k] * Pk_k_1[k][l] * C[j][l];
                }
            }
            Pyy[i][j] += R[i][j];
        }
    }

    // Kk = Pxy * inv(Pyy)
    float det_Pyy = Pyy[0][0] * Pyy[1][1] - Pyy[0][1] * Pyy[1][0];
    float inv_Pyy[2][2] = {
        {Pyy[1][1] / det_Pyy, -Pyy[0][1] / det_Pyy},
        {-Pyy[1][0] / det_Pyy, Pyy[0][0] / det_Pyy}
    };

    // Kk = Pxy * inv_Pyy
    float Kk[3][2] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
            Kk[i][j] = 0;
            for (int k = 0; k < 2; k++) {
                Kk[i][j] += Pxy[i][k] * inv_Pyy[k][j];
            }
        }
    }

    // xkk = xk_k_1 + Kk * (yk - ykk_1)
    float xkk[3];
    for (int i = 0; i < 3; i++) {
        xkk[i] = xk_k_1[i];
        for (int j = 0; j < 2; j++) {
            xkk[i] += Kk[i][j] * (yk[j] - ykk_1[j]);
        }
    }

    // Pkk = (I - Kk * C) * Pk_k_1
    float KkC[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 2; k++) {
                KkC[i][j] += Kk[i][k] * C[k][j];
            }
        }
    }

    float I_KkC[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            I_KkC[i][j] = (i == j ? 1.0f : 0.0f) - KkC[i][j];
        }
    }

    float Pkk[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                Pkk[i][j] += I_KkC[i][k] * Pk_k_1[k][j];
            }
        }
    }

    controller->vel_estimate = xkk[0];  
    controller->pos_estimate = xkk[1]; 
    controller->Tm = xkk[2];            

    //Low pass filter 200Hz
    float alpha = 0.005f;
	if (controller->Tm == 0) {
		controller->Tm_Filtered = controller->Tm;
	} else {
		controller->Tm_Filtered = alpha * controller->Tm + (1.0f - alpha) * controller->Tm_Filtered;
	}

    uk_1 = controller->Te;             
    for (int i = 0; i < 3; i++) {
        xk_1k_1[i] = xkk[i];
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Pk_1k_1[i][j] = Pkk[i][j];
        }
    }
}


