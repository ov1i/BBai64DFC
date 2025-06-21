#ifndef MPU9250_H
#define MPU9250_H

#include "i2c_helper.h"
extern "C" {
#include <stdint.h>
#include <stddef.h>
}
#include <utils.h>
#include <dfc_types.h>

#ifdef __cplusplus

namespace imu {
    class C_IMU {
    public:
        C_IMU() = default;
        ~C_IMU() = default;

        bool init();
        bool update();
        DFC_t_MPU9250_Data &getCurrentRawData();

    private:
        DFC_t_MPU9250_Data m_rawData;
        i2c::C_I2C m_i2cHandler;
};
} // namespace imu
#endif

#endif // MPU9250_H
