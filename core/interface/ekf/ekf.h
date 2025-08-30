#ifndef DFC_EKF_H
#define DFC_EKF_H

extern "C" {
#include <cstdint>
#include <cstring>
}

#include <cmath>
#include <dfc_types.h>
#include <shared_types.h>
#include <utils.h>

namespace ekf {

class C_EKF {
public:
  C_EKF() = default;

  void reset(uint64 t0_ns);
  void setParams(const DFC_t_EKF_Params &p) { m_params = p; }
  void setInitState(const DFC_t_EKF_State &state) { m_state = state; }
  const DFC_t_EKF_State& getState() const { return m_state; }

  // Low-level
  void predict(const float64 gyro_rad_s[3], const float64 acc_m_s2[3], float64 dt_s);
  
  void update_baro(float64 z_meas_NED_m, float64 std_override_m = -1.0);
  void update_mag_body(const float64 b_meas_body[3]); // normalized body mag vector
  void update_flow_pxrate_derot(const float64 px_rate_xy[2], float64 height_m, float64 quality = 1.0);

  // High-level (gyro in rad/s, accel in m/s^2, mag in uT, baro in Pa, altitude in m, flow is custom)
  void handle_imu(const DFC_t_MPU9250_Data& imu);
  void handle_mag(const DFC_t_MPU9250_Data& imu);
  void handle_baro(const DFC_t_BMP280_Data& baro);
  void handle_flow(const DFC_t_MsgOpticalFlow& msg);  // derotation included

  // High-level standstill init state
  bool graceful_state_init(float64 ax, float64 ay, float64 az,
                         float64 mx, float64 my, float64 mz, bool magRdy,
                         const float64 bg[3], float64 declination_rad,
                         uint64 t_ns);

  // Utility (get the rotation matrix according to the world frame, derot uses too)
  static void R_from_q(const float64 q[4], float64 R[9]);

private:
  DFC_t_EKF_Params m_params{};
  DFC_t_EKF_State  m_state{};

  // Time bookkeeping
  uint64 m_last_imu_ts_ns  { 0 };
  uint64 m_last_mag_ts_ns  { 0 };
  uint64 m_last_baro_ts_ns { 0 };
  uint64 m_last_flow_ts_ns { 0 };

  // IMU ring buffer for flow derotation
  DFC_t_GyroCorrect_Container m_gyroRing[GYRO_RING_SIZE]{};
  uint16 m_ringHead{0};

  // Helpers for accumulation gyro data over the frame duration and mapping low quality to big scale factor for low K gain (for derot)
  bool gyro_mean(uint64 t0, uint64 t1, float64 output[3]) const;
  void ring_push(uint64 ts, const float64 ringContainer[3]);
  static float64 getQualityScaleFactor(float64 quality);
  
  // Math helpers
  static void q_norm(float64 q[4]);
  static void q_mul(const float64 a[4], const float64 b[4], float64 o[4]);
  static void q_from_dtheta(const float64 dth[3], float64 q[4]);
  static void euler_from_accel(float64 ax, float64 ay, float64 az, float64& roll, float64& pitch);
  static bool yaw_from_accel_mag(float64 ax, float64 ay, float64 az, float64 mx, float64 my, float64 mz, float64 decl, float64& yaw_out);
  static void quat_from_euler_ZYX(float64 yaw, float64 pitch, float64 roll, float64 q[4]);

  // Small linear algebra helpers (float64 for covariance math)
  static void mat15_add_Q(float64* P, const float64* Q);
  static void mat15_FP_PFT(float64* P, const float64* F, float64 dt);
  static void matMN_vec(const float64* M, const float64* v, float64* out, uint8 rows, uint8 cols);
  static void axpy(float64* x, const float64* a, float64 k, uint8 n);

  // State correction and covariance housekeeping
  void inject_error(const float64 dx[15]);
  void pin_P();
};

} // namespace ekf

#endif // DFC_EKF_H
