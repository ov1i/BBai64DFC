#ifndef DFC_PID_H
#define DFC_PID_H

#include "pwm/PWMgen.h"

extern "C" {
#include <stdint.h>
#include <stddef.h>
#include <cstdint>
#include <cstring>
}

#include <cmath>
#include <algorithm>
#include <utils.h>
#include <dfc_types.h>

namespace ctrl {

class C_PIDController {
public:
  void init(const DFC_t_PIDController_Params& params);

  void update(
      const DFC_t_EKF_State& ekfState,      // pose/vel/quat
      const float64 gyroInput[3],       // body rates p,q,r from IMU
      const DFC_t_RcInputs& rcInput, float64 dt_s);

      const DFC_t_PIDControllerState& getState() const { return m_State; }
private:
  DFC_t_PIDController_Params m_Params;
  DFC_t_PIDControllerState m_State;

  // helpers
  static inline float64 apply_deadband_expo(float64 v, float64 deadband, float64 expo);
  static inline void quat_to_euler312(const float64 q[4], float64& yaw, float64& pitch, float64& roll);
  static inline void quat_conjugate(const float64 q[4], float64 out[4]);
  static inline void quat_mul(const float64 a[4], const float64 b[4], float64 out[4]);
  static inline void quat_to_axis_angle(const float64 q[4], float64 axis[3], float64& angle);

  // loops
  void position_hold_xy(const DFC_t_EKF_State& ekfState, const DFC_t_RcInputs& rc, float64 dt);
  void altitude_hold(const DFC_t_EKF_State& ekfState, const DFC_t_RcInputs& rc, float64 dt,
                     float64& thrust_cmd, float64& climb_rate_sp_out);

  void attitude_outer(const DFC_t_EKF_State& ekfState,
                      float64 tilt_sp_roll, float64 tilt_sp_pitch, float64 yaw_rate_sp,
                      float64 body_rate_sp[3]);

  void rate_inner(const float64 body_rate_sp[3], const float64 gyro[3], float64 dt,
                  float64 torque_cmd[3]);

  void mix_and_output(float64 thrust_cmd, const float64 torque_cmd[3], bool armed);

  // state helpers
  void capture_pos_sp_if_needed(const DFC_t_EKF_State& ekfState, const DFC_t_RcInputs& rc);
  static inline float64 constrain(float64 x, float64 lo, float64 hi) {
    return x < lo ? lo : (x > hi ? hi : x);
  }
};

} // namespace ctrl

#endif