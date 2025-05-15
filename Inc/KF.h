#ifndef KF_H
#define KF_H

#ifdef __cplusplus
extern "C" {
#endif


#include "structs.h"
#include "foc.h"
#include "math_ops.h"
#include "position_sensor.h"
void KF(ControllerStruct *controller, EncoderStruct *encoder);
float angle_difference(float angle1, float angle2);
#ifdef __cplusplus
}
#endif

#endif // MPC_TRAJECTORY_H
