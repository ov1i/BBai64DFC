#ifndef UTILS_H
#define UTILS_H
#include "data_types.h"

#define PADCONFIG_OFFSET_REG116             (116 * 4) // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG119             (119 * 4) // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG120             (120 * 4) // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG121             (121 * 4) // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG140             (140 * 4) // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG142             (142 * 4) // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG143             (143 * 4) // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG89              (89 * 4)  // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG90              (90 * 4)  // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG94              (94 * 4)  // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG95              (95 * 4)  // register number * 4 bytes per register

const uint8 MPU9250_ADDRESS =                0x68;

const uint8 MPU9250_WHO_AM_I_GENERAL =       0x75;
const uint8 MPU9250_PWR_MGMNT_1 =            0x6B;

const uint8  MPU9250_CONFIG =                0x1A;
const uint8  MPU9250_SMPRT_DIV =             0x19;
const uint8  MPU9250_GYRO_CONFIG =           0x1B;
const uint8  MPU9250_INT_PIN_CONFIG =        0x37;
const uint8  MPU9250_INT_ENABLE =            0x38;
const uint8  MPU9250_INT_STATUS =            0x3A;

const uint8  MPU9250_ACC_CONFIG_1 =          0x1C;
const uint8  MPU9250_ACC_CONFIG_2 =          0x1D;
const uint8  MPU9250_ACC_X_H =               0x3B;
const uint8  MPU9250_ACC_X_L =               0x3C;
const uint8  MPU9250_ACC_Y_H =               0x3D;
const uint8  MPU9250_ACC_Y_L =               0x3E;
const uint8  MPU9250_ACC_Z_H =               0x3F;
const uint8  MPU9250_ACC_Z_L =               0x40;

const uint8  MPU9250_TEMP_H =                0x41;
const uint8  MPU9250_TEMP_L =                0x42;

const uint8  MPU9250_GYRO_X_H =              0x43;
const uint8  MPU9250_GYRO_X_L =              0x44;
const uint8  MPU9250_GYRO_Y_H =              0x45;
const uint8  MPU9250_GYRO_Y_L =              0x46;
const uint8  MPU9250_GYRO_Z_H =              0x47;
const uint8  MPU9250_GYRO_Z_L =              0x48;

const uint8  MPU9250_USER_CTRL =             0x6A;
const uint8  MPU9250_SIG_PATH_RST =          0x68;

const uint8  MPU9250_AK8963_ADDR =           0x0C;
const uint8  MPU9250_AK8963_DEVICE_ID =      0x00;
const uint8  MPU9250_MAG_CONTROL_CONFIG =    0x0A;
const uint8  MPU9250_MAG_ASAX_CONFIG =       0x10;
const uint8  MPU9250_MAG_ASAY_CONFIG =       0x11;
const uint8  MPU9250_MAG_ASAZ_CONFIG =       0x12;
const uint8  MPU9250_MAG_STATUS_1 =          0x02;
const uint8  MPU9250_MAG_STATUS_2 =          0x09;
const uint8  MPU9250_MAG_X_L =               0x03;
const uint8  MPU9250_MAG_X_H =               0x04;
const uint8  MPU9250_MAG_Y_L =               0x05;
const uint8  MPU9250_MAG_Y_H =               0x06;
const uint8  MPU9250_MAG_Z_L =               0x07;
const uint8  MPU9250_MAG_Z_H =               0x08;

const uint8  BMP280_ADDRESS =                0x76;
const uint8  BMP280_RST_REG =                0xE0;
const uint8  BMP280_CTRL_MES_REG =           0xF4;
const uint8  BMP280_CONFIG_REG =             0xF5;
const uint8  BMP280_T1_CALIB_ADDR =          0x88;
const uint8  BMP280_PMSB_DATA_REGISTER =     0xF7;
const uint8  BMP280_STATUS_REGISTER =        0xF3;

const float64 ACC_LSB_PER_G =                8192.0;
const float64 GYRO_LSB_PER_DPS =             65.5;
const float64 MAG_UT_PER_LSB =               0.15;
const float64 GRAVITY =                      9.80665;
const float64 DEG2RAD =                      0.0174532925;
const uint16  GYRO_RING_SIZE =               256;           // -> 256(samples) at 5ms => 256 * 5ms = 1.28s

const float64 acc_cvt_cst =                 GRAVITY / ACC_LSB_PER_G;
const float64 gyro_cvt_cst =                (1.0 / GYRO_LSB_PER_DPS) * DEG2RAD;

const uint32 INVALID_ENDPOINT =             0xFFFFu;
const uint32 INVALID_REMOTEPROC =           0xFFFFu;

#endif
