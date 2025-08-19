#ifndef MPU9250_H
#define MPU9250_H

#include "i2c/i2c_helper.h"
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
        C_IMU();
        ~C_IMU() = default;

        bool init();
        bool update();
        void getCurrentRawData(DFC_t_MPU9250_Data *rawData);

    private:
        DFC_t_MPU9250_Data m_rawData;
        i2c::C_I2C m_i2cHandler;
        bool m_i2cState;
};
} // namespace imu
#endif

#endif // MPU9250_H
