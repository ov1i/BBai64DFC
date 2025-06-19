#ifndef __DFC_TYPES_H
#define __DFC_TYPES_H

#include "data_types.h"

typedef struct {
    float32 ax;
    float32 ay;
    float32 az;
    float32 gx;
    float32 gy;
    float32 gz;
    float32 mx;
    float32 my;
    float32 mz;
} DFC_t_MPU9250_Data;

typedef struct {
    float32 X[7];       // Quaternion (q0, q1, q2, q3) and gyro bias (bgx, bgy, bgz)
    float32 P[7][7];    // Covariance matrix
} DFC_t_EKF_params;

#endif
