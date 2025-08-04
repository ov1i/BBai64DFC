#ifndef __DFC_TYPES_H
#define __DFC_TYPES_H

#include "data_types.h"

#ifdef __cplusplus
namespace imu {
    class C_IMU;
}

typedef struct {
    float64 ax, ay, az;                 // Accelerometer
    float64 gx, gy, gz;                 // Gyroscope
    float64 mx, my, mz;                 // Magnetometer
    float64 mag_adjustment[3];          // Magnetometer Sensibility
    bool mag_rdy = false;               // Magnetometer Ready flag
    float64 temp;                       // Temperature
} DFC_t_MPU9250_Data;

typedef struct {
    float64 pressure;       // Accelerometer
    // float64 temp;        // Temperature
} DFC_t_BMP280_Data;

typedef struct {
    float64 ax, ay, az;
    float64 gx, gy, gz;
    float64 mx, my, mz;
    float64 pressure;
    float64 temperature;
    uint64 timestamp_us;
    bool has_mag;
    bool has_baro;
} DFC_t_EKF_Input;

typedef struct {
    imu::C_IMU *imu;
    QueueP_Handle imu_data_queue;
    SemaphoreP_Handle imu_data_semaph;
} DFC_t_IMU_TaskArgs;

typedef struct {
    // EKF *ekf;
    QueueP_Handle imu_data_queue;
    SemaphoreP_Handle imu_data_semaph;
} DFC_t_EKF_TaskArgs_t;

#endif

#endif