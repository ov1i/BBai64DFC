#ifndef __DFC_TYPES_H
#define __DFC_TYPES_H

#include "data_types.h"

typedef struct {
    float64 ax, ay, az;  // Accelerometer
    float64 gx, gy, gz;  // Gyroscope
    float64 mx, my, mz;  // Magnetometer
    // float64 ms[3];       // Magnetometer Sensibility
} DFC_t_MPU9250_Data;

typedef struct {
    float64 X[7];  // [q0, q1, q2, q3, bgx, bgy, bgz]  (Quaternion + Gyro Bias)
    float64 P[7][7];  // Covariance Matrix
} DFC_t_EKF_params;

#endif