#include "mpu9250.h"
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/csl_utils.h>

MPU9250::MPU9250(I2C_Handle handle, uint8_t addr) : i2cHandle(handle), devAddr(addr) {}

bool MPU9250::initialize() {
    return writeByte(0x6B, 0x00); // Wake up from sleep mode
}

bool MPU9250::readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
    uint8_t buffer[6];
    if (!readBytes(0x3B, buffer, 6)) return false;
    ax = ((int16_t)buffer[0] << 8) | buffer[1];
    ay = ((int16_t)buffer[2] << 8) | buffer[3];
    az = ((int16_t)buffer[4] << 8) | buffer[5];
    return true;
}

bool MPU9250::readGyro(int16_t &gx, int16_t &gy, int16_t &gz) {
    uint8_t buffer[6];
    if (!readBytes(0x43, buffer, 6)) return false;
    gx = ((int16_t)buffer[0] << 8) | buffer[1];
    gy = ((int16_t)buffer[2] << 8) | buffer[3];
    gz = ((int16_t)buffer[4] << 8) | buffer[5];
    return true;
}

bool MPU9250::readMag(int16_t &mx, int16_t &my, int16_t &mz) {
    return false;
}

bool MPU9250::writeByte(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2cWrite(i2cHandle, devAddr, data, 2);
}

bool MPU9250::readBytes(uint8_t reg, uint8_t *data, uint8_t len) {
    return i2cWriteRead(i2cHandle, devAddr, &reg, 1, data, len);
}
