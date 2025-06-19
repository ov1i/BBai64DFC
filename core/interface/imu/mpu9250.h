#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPU9250_I2C_ADDR         0x68
#define MPU9250_WHO_AM_I_ID      0x71

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
} mpu9250_raw_data_t;

// Initialize MPU9250 device
int mpu9250_init(void);

// Read raw data from MPU9250
int mpu9250_read_raw(mpu9250_raw_data_t *data);

// Perform a simple self-test or WHO_AM_I check
int mpu9250_check_id(void);

#ifdef __cplusplus
}
#endif

#endif // MPU9250_H
