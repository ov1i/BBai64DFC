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

        bool getCalibrationGyroBias(uint32 ms, float64 *bg);
        void getRawAccMagSample(float64 *accRaw, float64 *magRaw);
        
        void setParams(const DFC_t_MPU9250_Params& params) { m_params = params; }
        const DFC_t_MPU9250_Data& getData() const { return m_data; }
    private:
        // internal helpers
        bool readRawOnceForCalib(DFC_t_MPU9250_Data& data);
        static inline void mapValues(const float64 M[9], const float64 v[3], float64 out[3]) {
            out[0] = M[0]*v[0] + M[1]*v[1] + M[2]*v[2];
            out[1] = M[3]*v[0] + M[4]*v[1] + M[5]*v[2];
            out[2] = M[6]*v[0] + M[7]*v[1] + M[8]*v[2];
        }
        // !internal helpers
        
        // internal processing
        void processAcc(uint8 rbuffer[14]);
        void processMag(uint8 mag_data[7]);
        // !internal processing

        DFC_t_MPU9250_Data m_data;
        DFC_t_MPU9250_Params m_params;

        i2c::C_I2C m_i2cHandler;
        bool m_i2cState;
};
} // namespace imu
#endif

#endif // MPU9250_H
