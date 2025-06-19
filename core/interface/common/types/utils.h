#ifndef UTILS_H
#define UTILS_H
#include "data_types.h"

#define MPU9250_ADDRESS                 0x68

#define MPU9250_WHO_AM_I_GENERAL        0x75
#define MPU9250_PWR_MGMNT_1             0x6B

#define MPU9250_CONFIG                  0x1A
#define MPU9250_SMPRT_DIV               0x19
#define MPU9250_GYRO_CONFIG             0x1B
#define MPU9250_INT_PIN_CONFIG          0x37
#define MPU9250_INT_ENABLE              0x38
#define MPU9250_INT_STATUS              0x3A

#define MPU9250_ACC_CONFIG_1            0x1C
#define MPU9250_ACC_CONFIG_2            0x1D
#define MPU9250_ACC_X_H                 0x3B
#define MPU9250_ACC_X_L                 0x3C
#define MPU9250_ACC_Y_H                 0x3D
#define MPU9250_ACC_Y_L                 0x3E
#define MPU9250_ACC_Z_H                 0x3F
#define MPU9250_ACC_Z_L                 0x40

#define MPU9250_TEMP_H                  0x41
#define MPU9250_TEMP_L                  0x42

#define MPU9250_GYRO_X_H                0x43
#define MPU9250_GYRO_X_L                0x44
#define MPU9250_GYRO_Y_H                0x45
#define MPU9250_GYRO_Y_L                0x46
#define MPU9250_GYRO_Z_H                0x47
#define MPU9250_GYRO_Z_L                0x48

#define MPU9250_USER_CTRL               0x6A
#define MPU9250_SIG_PATH_RST            0x68

#define MPU9250_AK8963_ADDR             0x0C
#define MPU9250_AK8963_DEVICE_ID        0x00
#define MPU9250_MAG_CONTROL_CONFIG      0x0A
#define MPU9250_MAG_ASAX_CONFIG         0x10
#define MPU9250_MAG_ASAY_CONFIG         0x11
#define MPU9250_MAG_ASAZ_CONFIG         0x12
#define MPU9250_MAG_STATUS_1            0x02
#define MPU9250_MAG_STATUS_2            0x09
#define MPU9250_MAG_X_L                 0x03
#define MPU9250_MAG_X_H                 0x04
#define MPU9250_MAG_Y_L                 0x05
#define MPU9250_MAG_Y_H                 0x06
#define MPU9250_MAG_Z_L                 0x07
#define MPU9250_MAG_Z_H                 0x08

extern const float32 accel_scale;
extern const float32 gyro_scale;
extern const float64 deg_2_rad_cst;

#endif