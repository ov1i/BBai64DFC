#ifndef __EKH_INTERFACE_H
#define __EKH_INTERFACE_H

#include "dfc_types.h"
#include "data_types.h"
#include "string.h"
#include "math.h"
#include "utils.h"

namespace ekh {
    class ekh_interface {
        public:
            ekh_interface() = default;
            ~ekh_interface() = default;
            void init();
            void estimate(DFC_t_MPU9250_Data &imuData, float32 &dt);
            DFC_t_EKF_params getEKF() { return m_ekfParams; }
        private:
            void update(DFC_t_MPU9250_Data &imuData);
            void predict(DFC_t_MPU9250_Data &imuData, float32 &dt);
            void normalizeQuat();
            void cvtQuat2Euler();  // optional if needed
        private:
            DFC_t_EKF_params m_ekfParams;
    };
}   // namespace ekh

#endif
