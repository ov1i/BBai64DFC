#ifndef DFC_TYPES_H
#define DFC_TYPES_H

#include "data_types.h"

#ifdef __cplusplus
namespace imu {
class C_IMU;
}

typedef struct {
  float64 ax, ay, az;        // Accelerometer
  float64 gx, gy, gz;        // Gyroscope
  float64 mx, my, mz;        // Magnetometer
  float64 mag_adjustment[3]; // Magnetometer Sensibility
  bool mag_rdy;              // Magnetometer Ready flag
  float64 temp;              // Temperature
  uint64 ts_ns;              // Timestamp IMU(Accelerometer+Gyro)
  uint64 tsmag_ns;           // Timestamp IMU(Magnetometer)
} DFC_t_MPU9250_Data;

typedef struct {
  float64 P0;
  float64 P0_sum;
  uint32 P0_count;
} DFC_t_BMP280_GND_REF;

typedef struct {
  uint16 dig_T1;
  sint16 dig_T2, dig_T3;
  uint16 dig_P1;
  sint16 dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
} DFC_t_BMP280_Calib;

typedef struct {
  float64 pressure;                    // Pressure [Pa]
  float64 altitude;                    // Altitude [m]
  float64 temp;                        // Temperature [C]
  DFC_t_BMP280_GND_REF gnd;            // Ground reference
  DFC_t_BMP280_Calib calibration_data; // Calibration data
  uint64 ts_ns;
} DFC_t_BMP280_Data;

struct DFC_t_EKF_Params{
  // Continuous-time noise densities (tune):
  float32 gyro_noise = 0.012F;    // rad/s/√Hz
  float32 acc_noise = 0.25F;      // m/s^2/√Hz
  float32 gyro_bias_rw = 0.0005F; // rad/s^2/√Hz
  float32 acc_bias_rw = 0.01F;    // m/s^3/√Hz

  // Measurement std dev:
  float32 baro_std = 0.7F; // m (NED z)
  float32 mag_std = 0.8F; // arbitrary vector std [uT or normalized]; tune after calib
  float32 flow_vel_std = 0.25F; // m/s at ~1 m height; scaled with height below

  // Earth magnetic field in NED (normalized or in uT) – set after a quick
  // on-ground calibration
  float32 magN[3] = {0.25f, 0.02f, 0.43f}; // direction, magnitude not critical if std is large
};

typedef struct {
  // Nominal state
  float64 p[3];  // NED position (m)
  float64 v[3];  // NED velocity (m/s)
  float64 q[4];  // quaternion body->NED [w,x,y,z]
  float64 bg[3]; // gyro bias (rad/s)
  float64 ba[3]; // accel bias (m/s^2)

  // Error-state covariance P (15x15), row-major
  float64 P[15 * 15];

  uint64 t_ns;
} DFC_t_EKF_State;

enum class DFC_t_Mode : uint8 {
  ACRO = 0,       // rate control only
  ANGLE = 1,      // attitude hold (roll/pitch angles + yaw rate)
  POS_HOLD = 2    // XY position + altitude hold with recovery
};

typedef struct {
  // normalized sticks in [-1,1] except throttle in [0,1]
  float64 thr;          // 0..1 collective
  float64 roll;         // -1..1
  float64 pitch;        // -1..1
  float64 yaw;          // -1..1
  bool arm;             // true when armed
  DFC_t_Mode mode;      // flight mode
} DFC_t_RcInputs;

struct DFC_t_RcParams {
    float64  ecap_clk_hz = 100e6;

    // Limits
    float64  thr_min_us   = 1000.0;
    float64  thr_max_us   = 2000.0;
    float64  center_us    = 1500.0;
    float64  span_half_us = 500.0;

    // Shaping
    float64  deadband  = 0.01; // ~0.0 -> 0.02 (upper limit should not be passed mushy control)
    float64  lpf_alpha = 0.20;

    // Failsafe
    uint32 timeout_ms = 120;

    // Default mode until we set it with set_mode()
    DFC_t_Mode default_mode = DFC_t_Mode::POS_HOLD;

    // In case we want to switch the sign
    bool invert_roll  = false;
    bool invert_pitch = false;
  };

struct DFC_t_PIDController_Params {
  // Inner RATE PID (using gyro rad/s): torque commands in [-1,1]
  float64 kp_rate[3] = {0.15, 0.15, 0.12};
  float64 ki_rate[3] = {0.08, 0.08, 0.05};
  float64 kd_rate[3] = {0.0035, 0.0035, 0.0025};
  float64 rate_i_lim[3] = {0.25, 0.25, 0.15};     // integral clamps

  // Attitude P (quat error → body rate cmd) [rad/s per rad]
  float64 kp_att_roll  = 4.0;
  float64 kp_att_pitch = 4.0;
  float64 kp_att_yaw   = 2.5;

  // RC → targets
  float64 rc_max_rate[3] = {3.5, 3.5, 2.5};      // rad/s at full stick (ACRO or yaw in ANGLE/PH)
  float64 rc_max_tilt = 0.35;                      // rad (≈20°) angle command in ANGLE/PH
  float64 rc_yaw_rate = 2.0;                       // rad/s at full yaw stick in ANGLE/PH
  float64 rc_deadband = 0.06;                      // stick deadband
  float64 rc_expo = 0.3;                           // expo shaping

  // Position hold (horizontal)
  float64 kp_pos_xy = 1.2;     // m → m/s
  float64 ki_pos_xy = 0.25;    // m·s → m/s
  float64 vel_i_lim_xy = 1.0;  // m/s
  float64 kp_vel_xy = 0.8;     // (m/s) → (m/s^2)
  float64 ki_vel_xy = 0.2;     // (m/s·s) → (m/s^2)
  float64 acc_i_lim_xy = 2.0;  // m/s^2
  float64 max_vel_xy = 3.0;    // m/s
  float64 max_acc_xy = 4.0;    // m/s^2

  // Altitude hold (down is +Z in NED)
  float64 kp_pos_z = 1.6;      // m → m/s
  float64 ki_pos_z = 0.3;      // m·s → m/s
  float64 vel_i_lim_z = 1.0;
  float64 kp_vel_z = 1.2;      // (m/s) → (m/s^2)
  float64 ki_vel_z = 0.25;     // (m/s·s) → (m/s^2)
  float64 acc_i_lim_z = 3.0;   // m/s^2
  float64 max_vel_z_up   = 2.0; // m/s (up is negative NED Z)
  float64 max_vel_z_down = 1.5; // m/s (down is positive NED Z)
  float64 max_acc_z = 6.0;     // m/s^2

  // Hover/thrust model and PWM
  float64 hover_thrust = 0.38;   // normalized [0..1] to just hover (tune!)
  float64 min_thrust   = 0.05;   // floor to keep motors spinning when armed
  float64 max_thrust   = 0.95;   // cap
  // PWM microseconds mapping
  uint16 pwm_min_us     = 1000;
  uint16 pwm_max_us     = 2000;

  // Mixer gains (quad X)
  float64 mix_roll  = 1.0;
  float64 mix_pitch = 1.0;
  float64 mix_yaw   = 0.6;     // start smaller for yaw authority
};

struct DFC_t_PIDControllerState {
  // Position hold setpoints
  float64 pos_sp_NED[3] = {0,0,0};  // [N,E,D]
  float64 vel_sp_NED[3] = {0,0,0};  // desired velocities
  float64 yaw_sp = 0.0;                         // yaw target [rad]
  bool pos_sp_valid = false;

  // Integrators
  float64 i_rate[3] = {0,0,0};
  float64 i_vel_xy[2] = {0,0};
  float64 i_vel_z = 0.0;

  // For standstill recovery
  bool motors_saturated = false;

  // Arming
  bool isArmed = false;
};

struct DFC_t_PWMgen_Params {
  float64 ehrpwm_clk_hz  = 100e6;

  // Output PWM frame rate (ESCs are on 50 Hz for 1–2 ms pulses)
  uint32 freq_hz = 50;

  // Output limits (µs) -> Last line of defense in case all else fails
  uint32 min_us = 1000;
  uint32 max_us = 2000;
};

typedef struct {
  imu::C_IMU *imu;
  QueueP_Handle imu_data_queue;
  SemaphoreP_Handle imu_data_semaph;
} DFC_t_IMU_TaskArgs;

typedef struct {
  // EKF *ekf;
  QueueP_Handle imu_data_queue;
  SemaphoreP_Handle imu_data_semaph;
} DFC_t_EKF_TaskArgs;

#endif

#endif