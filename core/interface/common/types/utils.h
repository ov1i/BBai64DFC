#ifndef UTILS_H
#define UTILS_H
#include "data_types.h"

#define PADCONFIG_OFFSET_REG116                 (116 * 4) // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG119                 (119 * 4) // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG120                 (120 * 4) // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG121                 (121 * 4) // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG140                 (140 * 4) // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG142                 (142 * 4) // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG143                 (143 * 4) // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG89                  (89 * 4)  // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG90                  (90 * 4)  // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG94                  (94 * 4)  // register number * 4 bytes per register
#define PADCONFIG_OFFSET_REG95                  (95 * 4)  // register number * 4 bytes per register

#define MPU9250_ADDRESS                         ((uint8)0x68)

#define MPU9250_WHO_AM_I_GENERAL                ((uint8)0x75)
#define MPU9250_PWR_MGMNT_1                     ((uint8)0x6B)

#define MPU9250_CONFIG                          ((uint8)0x1A)
#define MPU9250_SMPRT_DIV                       ((uint8)0x19)
#define MPU9250_GYRO_CONFIG                     ((uint8)0x1B)
#define MPU9250_INT_PIN_CONFIG                  ((uint8)0x37)
#define MPU9250_INT_ENABLE                      ((uint8)0x38)
#define MPU9250_INT_STATUS                      ((uint8)0x3A)

#define MPU9250_ACC_CONFIG_1                    ((uint8)0x1C)
#define MPU9250_ACC_CONFIG_2                    ((uint8)0x1D)
#define MPU9250_ACC_X_H                         ((uint8)0x3B)
#define MPU9250_ACC_X_L                         ((uint8)0x3C)
#define MPU9250_ACC_Y_H                         ((uint8)0x3D)
#define MPU9250_ACC_Y_L                         ((uint8)0x3E)
#define MPU9250_ACC_Z_H                         ((uint8)0x3F)
#define MPU9250_ACC_Z_L                         ((uint8)0x40)

#define MPU9250_TEMP_H                          ((uint8)0x41)
#define MPU9250_TEMP_L                          ((uint8)0x42)

#define MPU9250_GYRO_X_H                        ((uint8)0x43)
#define MPU9250_GYRO_X_L                        ((uint8)0x44)
#define MPU9250_GYRO_Y_H                        ((uint8)0x45)
#define MPU9250_GYRO_Y_L                        ((uint8)0x46)
#define MPU9250_GYRO_Z_H                        ((uint8)0x47)
#define MPU9250_GYRO_Z_L                        ((uint8)0x48)

#define MPU9250_USER_CTRL                       ((uint8)0x6A)
#define MPU9250_SIG_PATH_RST                    ((uint8)0x68)

#define MPU9250_AK8963_ADDR                     ((uint8)0x0C)
#define MPU9250_AK8963_DEVICE_ID                ((uint8)0x00)
#define MPU9250_MAG_CONTROL_CONFIG              ((uint8)0x0A)
#define MPU9250_MAG_ASAX_CONFIG                 ((uint8)0x10)
#define MPU9250_MAG_ASAY_CONFIG                 ((uint8)0x11)
#define MPU9250_MAG_ASAZ_CONFIG                 ((uint8)0x12)
#define MPU9250_MAG_STATUS_1                    ((uint8)0x02)
#define MPU9250_MAG_STATUS_2                    ((uint8)0x09)
#define MPU9250_MAG_X_L                         ((uint8)0x03)
#define MPU9250_MAG_X_H                         ((uint8)0x04)
#define MPU9250_MAG_Y_L                         ((uint8)0x05)
#define MPU9250_MAG_Y_H                         ((uint8)0x06)
#define MPU9250_MAG_Z_L                         ((uint8)0x07)
#define MPU9250_MAG_Z_H                         ((uint8)0x08)

#define BMP280_ADDRESS                          ((uint8)0x76)
#define BMP280_RST_REG                          ((uint8)0xE0)
#define BMP280_CTRL_MES_REG                     ((uint8)0xF4)
#define BMP280_CONFIG_REG                       ((uint8)0xF5)
#define BMP280_T1_CALIB_ADDR                    ((uint8)0x88)
#define BMP280_PMSB_DATA_REGISTER               ((uint8)0xF7)
#define BMP280_STATUS_REGISTER                  ((uint8)0xF3)

#define ACC_LSB_PER_G                           ((float64)8192.0)
#define GYRO_LSB_PER_DPS                        ((float64)65.5)
#define MAG_UT_PER_LSB                          ((float64)0.15)
#define GRAVITY                                 ((float64)9.80665)
#define DEG2RAD                                 ((float64)0.017453292519943295)
#define GYRO_RING_SIZE                          ((uint16)256U)          // -> 256(samples) at 5ms > 256 * 5ms  1.28s

#define acc_cvt_cst                             ((float64)(((GRAVITY) / (ACC_LSB_PER_G))))
#define gyro_cvt_cst                            ((float64)((DEG2RAD) / (GYRO_LSB_PER_DPS)))

#define INVALID_ENDPOINT                        ((uint32)0xFFFFu)
#define INVALID_REMOTEPROC                      ((uint32)0xFFFFu)

#endif
