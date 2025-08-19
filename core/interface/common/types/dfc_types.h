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
    bool mag_rdy;                       // Magnetometer Ready flag
    float64 temp;                       // Temperature
    uint64 ts_ns;                       // Timestamp IMU(Accelerometer+Gyro)
    uint64 tsmag_ns;                    // Timestamp IMU(Magnetometer)
} DFC_t_MPU9250_Data;

typedef struct {
    float64 P0;
    float64 P0_sum;
    uint32  P0_count;
} DFC_t_BMP280_GND_REF;

typedef struct {
    uint16 dig_T1;  
    sint16 dig_T2, dig_T3;
    uint16 dig_P1; 
    sint16 dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} DFC_t_BMP280_Calib;

typedef struct {
    uint64 ts_ns;
    float64 pressure;                       // Pressure [Pa]
    float64 altitude;                       // Altitude [m]
    float64 temp;                           // Temperature [C]
    DFC_t_BMP280_GND_REF gnd;               // Ground reference
    DFC_t_BMP280_Calib calibration_data;    // Calibration data
} DFC_t_BMP280_Data;

typedef struct {
    float64 ax, ay, az;
    float64 gx, gy, gz;
    float64 mx, my, mz;
    float64 pressure;
    float64 temperature;
    uint64 timestamp_us;
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
} DFC_t_EKF_TaskArgs;

#endif

#endif