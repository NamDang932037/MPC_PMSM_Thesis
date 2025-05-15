#ifndef MPC_TRAJECTORY_H
#define MPC_TRAJECTORY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "structs.h"
#include "foc.h"

void CreateSystemMatrices(float32_t* Ae, float32_t* Be, float32_t* Ce);
void createPredictionMatrices(
    const float32_t* A_data,
    const float32_t* B_data,
    const float32_t* C_data,
    float32_t* F_data,
    float32_t* G_data);
void createConstraintMatrices(ControllerStruct *controller, float32_t* M_data, float32_t* N_data);
void planStepTrajectory(ControllerStruct *controller, struct TrajectoryPoint* trajectory);
void planSCurveTrajectory(ControllerStruct *controller, struct TrajectoryPoint* trajectory);
void planSineTrajectory(ControllerStruct *controller, struct TrajectoryPoint* trajectory);
void planLSPBTrajectory(ControllerStruct *controller, struct TrajectoryPoint* trajectory);
void calculateQPMatrices(
	const float32_t* G_data,
	const float32_t* Qy_data,
	const float32_t* R_u_data,
	const float32_t* F_data,
	const float32_t* X_data,
	const float32_t* Rs_data,
	float32_t* H_data,
	float32_t* f_data);
void createReferenceVector(ControllerStruct *controller, float32_t* Rs);
void solveHildreth(const float32_t* H_data, const float32_t* f_data,
                   const float32_t* M_data, const float32_t* N_data,
                   float32_t* delta_U);

void MPC_Calculate(ControllerStruct *controller);

#ifdef __cplusplus
}
#endif

#endif // MPC_TRAJECTORY_H
