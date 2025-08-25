#ifndef __DFC_TYPES_H
#define __DFC_TYPES_H

#include "data_types.h"

#ifdef __cplusplus
namespace imu {
class C_IMU;
}

typedef struct {
  float64 ax, ay, az;        // Accelerometer
  float64 gx, gy, gz;        // Gyroscope
  float64 mx, my, mz;        // Magnetometer
  float64 mag_adjustment[3]; // Magnetometer Sensibility
  bool mag_rdy;              // Magnetometer Ready flag
  float64 temp;              // Temperature
  uint64 ts_ns;              // Timestamp IMU(Accelerometer+Gyro)
  uint64 tsmag_ns;           // Timestamp IMU(Magnetometer)
} DFC_t_MPU9250_Data;

typedef struct {
  float64 P0;
  float64 P0_sum;
  uint32 P0_count;
} DFC_t_BMP280_GND_REF;

typedef struct {
  uint16 dig_T1;
  sint16 dig_T2, dig_T3;
  uint16 dig_P1;
  sint16 dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} DFC_t_BMP280_Calib;

typedef struct {
  float64 pressure;                    // Pressure [Pa]
  float64 altitude;                    // Altitude [m]
  float64 temp;                        // Temperature [C]
  DFC_t_BMP280_GND_REF gnd;            // Ground reference
  DFC_t_BMP280_Calib calibration_data; // Calibration data
  uint64 ts_ns;
} DFC_t_BMP280_Data;

struct DFC_t_EKF_Params{
  // Continuous-time noise densities (tune):
  float32 gyro_noise = 0.012F;    // rad/s/√Hz
  float32 acc_noise = 0.25F;      // m/s^2/√Hz
  float32 gyro_bias_rw = 0.0005F; // rad/s^2/√Hz
  float32 acc_bias_rw = 0.01F;    // m/s^3/√Hz

  // Measurement std dev:
  float32 baro_std = 0.7F; // m (NED z)
  float32 mag_std = 0.8F; // arbitrary vector std [uT or normalized]; tune after calib
  float32 flow_vel_std = 0.25F; // m/s at ~1 m height; scaled with height below

  // Earth magnetic field in NED (normalized or in uT) – set after a quick
  // on-ground calibration
  float32 magN[3] = {0.25f, 0.02f, 0.43f}; // direction, magnitude not critical if std is large
};

typedef struct {
  // Nominal state
  float64 p[3];  // NED position (m)
  float64 v[3];  // NED velocity (m/s)
  float64 q[4];  // quaternion body->NED [w,x,y,z]
  float64 bg[3]; // gyro bias (rad/s)
  float64 ba[3]; // accel bias (m/s^2)

  // Error-state covariance P (15x15), row-major
  float64 P[15 * 15];

  // Time of last propagation
  uint64 t_ns;
} DFC_t_EKF_State;

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