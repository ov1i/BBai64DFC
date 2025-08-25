#include "controller/PIDController.h"
#include "pwm/PWMgen.h"


namespace ctrl {

void C_PIDController::init(const DFC_t_PIDController_Params& params) {
  m_Params = params;
  m_State = DFC_t_PIDControllerState{};
  /// TODO: MOVE TO INIT TASK
  PWMgen::init();
}

float64 C_PIDController::apply_deadband_expo(float64 v, float64 deadband, float64 expo) {
  // deadband
  if (std::fabs(v) < deadband) return 0.0;
  // rescale after DB so  (deadband..1) -> (0..1)
  float64 s = (std::fabs(v) - deadband) / (1.0 - deadband);
  s = std::clamp(s, 0.0, 1.0);
  // expo (smooth around center)
  float64 s_ex = (1.0 - expo) * s + expo * s * s * s;
  return (v >= 0 ? s_ex : -s_ex);
}

void C_PIDController::quat_conjugate(const float64 q[4], float64 out[4]) {
  out[0] = q[0]; out[1] = -q[1]; out[2] = -q[2]; out[3] = -q[3];
}
void C_PIDController::quat_mul(const float64 a[4], const float64 b[4], float64 o[4]) {
  o[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  o[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  o[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  o[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

void C_PIDController::quat_to_euler312(const float64 q[4], float64& yaw, float64& pitch, float64& roll) {
  // Convert to rotation matrix and extract nominal yaw (ψ), pitch(θ), roll(φ)
  const float64 w=q[0], x=q[1], y=q[2], z=q[3];
  // standard DCM
  float64 R00 = w*w + x*x - y*y - z*z;
  float64 R01 = 2*(x*y - w*z);
  float64 R02 = 2*(x*z + w*y);
  float64 R10 = 2*(x*y + w*z);
  float64 R11 = w*w - x*x + y*y - z*z;
  float64 R12 = 2*(y*z - w*x);
  float64 R20 = 2*(x*z - w*y);
  float64 R21 = 2*(y*z + w*x);
  float64 R22 = w*w - x*x - y*y + z*z;

  // NED yaw from R10/R00 (atan2(E,N))
  yaw   = std::atan2((float64)R10, (float64)R00);
  // Pitch (rotation about body-x approx). Use asin(-R20)
  pitch = std::asin((float64)(-R20));
  // Roll from R21/R22 (atan2)
  roll  = std::atan2((float64)R21, (float64)R22);
}

void C_PIDController::quat_to_axis_angle(const float64 q[4], float64 axis[3], float64& angle) {
  // assumes q normalized
  angle = (float64)(2.0 * std::acos(std::clamp(q[0], -1.0, 1.0)));
  float64 s = std::sqrt(std::max(1.0 - q[0]*q[0], 1e-12));
  axis[0] = (float64)(q[1] / s);
  axis[1] = (float64)(q[2] / s);
  axis[2] = (float64)(q[3] / s);
}

void C_PIDController::update(const DFC_t_EKF_State& ekfState,
                        const float64 gyro[3],
                        const DFC_t_RcInputs& rc_in,
                        float64 dt) {
  // Arm/Disarm edge handling
  m_State.isArmed = rc_in.arm;

  // RC shaping
  DFC_t_RcInputs rc = rc_in; // FAILSAFE LCOAL COPY

  /// TODO: Deadband cleanup stacked, should be single stack..? -> sol: move to single deadband, but we keep for now low RC deadband compensation
  rc.roll  = apply_deadband_expo(rc.roll,  m_Params.rc_deadband, m_Params.rc_expo);
  rc.pitch = apply_deadband_expo(rc.pitch, m_Params.rc_deadband, m_Params.rc_expo);
  rc.yaw   = apply_deadband_expo(rc.yaw,   m_Params.rc_deadband, m_Params.rc_expo);

  // throttle deadband around min to avoid twitch
  if (rc.thr < 0.02) rc.thr = 0.0;

  // Capture/maintain position setpoint in POS_HOLD
  capture_pos_sp_if_needed(ekfState, rc);

  // Build setpoints & loops
  float64 body_rate_sp[3] = {0,0,0};
  float64 thrust_cmd = 0.0;

  if (rc.mode == DFC_t_Mode::ACRO) {
    // ACRO: directly rate control
    body_rate_sp[0] = rc.roll  * m_Params.rc_max_rate[0];
    body_rate_sp[1] = rc.pitch * m_Params.rc_max_rate[1];
    body_rate_sp[2] = rc.yaw   * m_Params.rc_max_rate[2];

    // collective thrust from throttle stick (no altitude hold)
    thrust_cmd = m_Params.min_thrust + rc.thr * (m_Params.max_thrust - m_Params.min_thrust);

  } else if (rc.mode == DFC_t_Mode::ANGLE) {
    // ANGLE: attitude outer (roll/pitch angles), yaw rate, manual thrust
    float64 tilt_sp_roll  =  rc.roll  * m_Params.rc_max_tilt;
    float64 tilt_sp_pitch = -rc.pitch * m_Params.rc_max_tilt; // stick forward → pitch down
    float64 yaw_rate_sp   =  rc.yaw   * m_Params.rc_yaw_rate;

    attitude_outer(ekfState, tilt_sp_roll, tilt_sp_pitch, yaw_rate_sp, body_rate_sp);

    thrust_cmd = m_Params.min_thrust + rc.thr * (m_Params.max_thrust - m_Params.min_thrust);

  } else { // POS_HOLD
    // altitude hold & XY position hold
    float64 climb_rate_sp = 0.0;
    altitude_hold(ekfState, rc, dt, thrust_cmd, climb_rate_sp);
    position_hold_xy(ekfState, rc, dt);

    // turn horizontal acc command into roll/pitch targets:
    // We convert vel_sp from m_State.vel_sp_NED through a PI to acc_cmd, then to tilt inside position_hold_xy().
    // position_hold_xy stored desired tilt in vel_sp_NED[0..1] temporarily as tilt targets (trick to avoid another struct).
    float64 tilt_sp_roll  = (float64)m_State.vel_sp_NED[1];   // we placed roll tilt setpoint here
    float64 tilt_sp_pitch = (float64)m_State.vel_sp_NED[0];   // and pitch tilt setpoint here
    // yaw
    float64 yaw, pitch, roll;
    quat_to_euler312(ekfState.q, yaw, pitch, roll);
    float64 yaw_err = std::atan2(std::sin((float64)m_State.yaw_sp - yaw), std::cos((float64)m_State.yaw_sp - yaw));
    float64 yaw_rate_sp = m_Params.kp_att_yaw * yaw_err + rc.yaw * m_Params.rc_yaw_rate;

    attitude_outer(ekfState, tilt_sp_roll, tilt_sp_pitch, yaw_rate_sp, body_rate_sp);
  }

  // Inner rate PID (gyro)
  float64 torque_cmd[3];
  rate_inner(body_rate_sp, gyro, dt, torque_cmd);

  // Mix + send PWM
  mix_and_output(thrust_cmd, torque_cmd, m_State.isArmed);
}

void C_PIDController::capture_pos_sp_if_needed(const DFC_t_EKF_State& ekfState, const DFC_t_RcInputs& rc) {
  float64 yaw, pitch, roll;
  quat_to_euler312(ekfState.q, yaw, pitch, roll);

  // if we enter POS_HOLD or haven't set sp → capture
  if (rc.mode == DFC_t_Mode::POS_HOLD && !m_State.pos_sp_valid) {
    m_State.pos_sp_NED[0] = ekfState.p[0];
    m_State.pos_sp_NED[1] = ekfState.p[1];
    m_State.pos_sp_NED[2] = ekfState.p[2];
    m_State.yaw_sp = yaw;
    m_State.pos_sp_valid = true;
    m_State.i_vel_xy[0] = m_State.i_vel_xy[1] = 0;
    m_State.i_vel_z = 0;
  }
  if (rc.mode != DFC_t_Mode::POS_HOLD) {
    m_State.pos_sp_valid = false;
  }
}

void C_PIDController::position_hold_xy(const DFC_t_EKF_State& ekfState, const DFC_t_RcInputs& rc, float64 dt) {
  // stick → velocity setpoint offset
  const float64 vx_cmd = rc.roll  * m_Params.max_vel_xy;   // right stick -> +E
  const float64 vy_cmd = -rc.pitch * m_Params.max_vel_xy;  // forward stick -> +N

  // If sticks out of deadband, shift pos setpoint with velocity command
  const bool move = (std::fabs(rc.roll) > m_Params.rc_deadband) || (std::fabs(rc.pitch) > m_Params.rc_deadband);
  if (move) {
    // move the position target with commanded velocity
    m_State.pos_sp_NED[0] += (float64)(vy_cmd * dt);
    m_State.pos_sp_NED[1] += (float64)(vx_cmd * dt);
  }

  // Position error (N,E)
  float64 eN = (float64)(m_State.pos_sp_NED[0] - ekfState.p[0]);
  float64 eE = (float64)(m_State.pos_sp_NED[1] - ekfState.p[1]);

  // Outer m_Params + I -> velocity setpoint
  m_State.vel_sp_NED[0] = std::clamp(m_Params.kp_pos_xy * eN + m_State.i_vel_xy[0], -m_Params.max_vel_xy, m_Params.max_vel_xy);
  m_State.vel_sp_NED[1] = std::clamp(m_Params.kp_pos_xy * eE + m_State.i_vel_xy[1], -m_Params.max_vel_xy, m_Params.max_vel_xy);

  // Integrate only when not saturated
  if (!m_State.motors_saturated) {
    m_State.i_vel_xy[0] += m_Params.ki_pos_xy * eN * dt;
    m_State.i_vel_xy[1] += m_Params.ki_pos_xy * eE * dt;
    m_State.i_vel_xy[0] = constrain(m_State.i_vel_xy[0], -m_Params.vel_i_lim_xy, m_Params.vel_i_lim_xy);
    m_State.i_vel_xy[1] = constrain(m_State.i_vel_xy[1], -m_Params.vel_i_lim_xy, m_Params.vel_i_lim_xy);
  }

  // Velocity error → required horizontal acceleration (N,E)
  float64 evN = (float64)m_State.vel_sp_NED[0] - (float64)ekfState.v[0];
  float64 evE = (float64)m_State.vel_sp_NED[1] - (float64)ekfState.v[1];

  static float64 acc_i_N = 0, acc_i_E = 0;
  if (!m_State.motors_saturated) {
    acc_i_N += m_Params.ki_vel_xy * evN * dt;
    acc_i_E += m_Params.ki_vel_xy * evE * dt;
    acc_i_N = constrain(acc_i_N, -m_Params.acc_i_lim_xy, m_Params.acc_i_lim_xy);
    acc_i_E = constrain(acc_i_E, -m_Params.acc_i_lim_xy, m_Params.acc_i_lim_xy);
  }

  float64 accN = m_Params.kp_vel_xy * evN + acc_i_N;
  float64 accE = m_Params.kp_vel_xy * evE + acc_i_E;

  // limit
  float64 acc_norm = std::sqrt(accN*accN + accE*accE);
  if (acc_norm > m_Params.max_acc_xy) {
    float64 sc = m_Params.max_acc_xy / (acc_norm + 1e-6);
    accN *= sc; accE *= sc;
  }

  // Convert horizontal acc command → tilt targets (small angle, yaw-compensated)
  float64 yaw, pitch, roll;
  quat_to_euler312(ekfState.q, yaw, pitch, roll);
  // rotate N,E into body‑frame axes for tilt:
  // desired pitch (θ, nose down positive) ≈ -accN / g in earth, but account yaw
  float64 cY = std::cos(yaw), sY = std::sin(yaw);
  float64 accXb =  cY*accN + sY*accE;    // forward body
  float64 accYb = -sY*accN + cY*accE;    // right body

  float64 tilt_pitch_sp = -accXb / 9.80665; // rad approx
  float64 tilt_roll_sp  =  accYb / 9.80665; // rad approx

  tilt_pitch_sp = constrain(tilt_pitch_sp, -m_Params.rc_max_tilt, m_Params.rc_max_tilt);
  tilt_roll_sp  = constrain(tilt_roll_sp,  -m_Params.rc_max_tilt, m_Params.rc_max_tilt);

  // store into m_State.vel_sp_NED as a small hack to pass to attitude_outer()
  m_State.vel_sp_NED[0] = tilt_pitch_sp;
  m_State.vel_sp_NED[1] = tilt_roll_sp;
}

void C_PIDController::altitude_hold(const DFC_t_EKF_State& ekfState, const DFC_t_RcInputs& rc, float64 dt,
                               float64& thrust_cmd, float64& climb_rate_sp_out) {
  // On first enter, capture altitude SP
  if (!m_State.pos_sp_valid) {
    m_State.pos_sp_NED[2] = ekfState.p[2];
  }

  // Throttle stick → climb rate around zero with deadband
  // center stick ~ hover, up negative climb (NED down is +)
  float64 stick = apply_deadband_expo(rc.yaw*0.0f + rc.pitch*0.0f + rc.roll*0.0f, 0, 0); (void)stick;
  float64 t = rc.thr; // 0..1
  float64 climb_rate_sp = 0.0;
  const float64 mid = 0.5;
  const float64 db  = 0.05;

  if (t > mid + db) {
    float64 u = (t - (mid + db)) / (1.0 - (mid + db));
    climb_rate_sp = -u * m_Params.max_vel_z_up;    // up -> negative NED
  } else if (t < mid - db) {
    float64 u = ((mid - db) - t) / (mid - db);
    climb_rate_sp =  u * m_Params.max_vel_z_down;  // down -> positive NED
  } else {
    climb_rate_sp = 0.0;
  }
  climb_rate_sp_out = climb_rate_sp;

  // Position m_Params+I to make a vel_sp_z
  float64 ez = (float64)(m_State.pos_sp_NED[2] - ekfState.p[2]); // positive if we are above target? (NED down)
  float64 vel_sp_z = std::clamp(m_Params.kp_pos_z * ez + m_State.i_vel_z, -m_Params.max_vel_z_up, m_Params.max_vel_z_down);
  // override by stick climb if out of deadband
  if (std::fabs(climb_rate_sp) > 1e-3) vel_sp_z = climb_rate_sp;

  if (!m_State.motors_saturated) {
    m_State.i_vel_z += m_Params.ki_pos_z * ez * dt;
    m_State.i_vel_z = constrain(m_State.i_vel_z, -m_Params.vel_i_lim_z, m_Params.vel_i_lim_z);
  }

  // Velocity error → vertical acceleration (down positive)
  float64 evz = vel_sp_z - (float64)ekfState.v[2];
  static float64 acc_i_z = 0.0;
  if (!m_State.motors_saturated) {
    acc_i_z += m_Params.ki_vel_z * evz * dt;
    acc_i_z = constrain(acc_i_z, -m_Params.acc_i_lim_z, m_Params.acc_i_lim_z);
  }
  float64 accZ = m_Params.kp_vel_z * evz + acc_i_z;

  // Convert to thrust command: thrust ≈ hover + accZ/g
  float64 thrust = m_Params.hover_thrust + accZ / 9.80665;
  thrust_cmd = constrain(thrust, m_Params.min_thrust, m_Params.max_thrust);
}

void C_PIDController::attitude_outer(const DFC_t_EKF_State& ekfState,
                                float64 tilt_sp_roll, float64 tilt_sp_pitch, float64 yaw_rate_sp,
                                float64 body_rate_sp[3]) {
  // Build desired quaternion from roll/pitch setpoints and current yaw (yaw not directly commanded here)
  float64 yaw, pitch_c, roll_c;
  quat_to_euler312(ekfState.q, yaw, pitch_c, roll_c);

  float64 cr = std::cos(tilt_sp_roll*0.5),  sr = std::sin(tilt_sp_roll*0.5);
  float64 cp = std::cos(tilt_sp_pitch*0.5), sp = std::sin(tilt_sp_pitch*0.5);
  float64 cy = std::cos(yaw*0.5),           sy = std::sin(yaw*0.5);

  // compose q_sp = Rz(yaw)*Rx(pitch_sp)*Ry(roll_sp) approximately (order chosen to track)
  float64 q_sp[4];
  // yaw * pitch * roll (Z*X*Y)
  float64 q_yaw[4]   = {cy, 0, 0, sy};
  float64 q_pitch[4] = {cp, sp, 0, 0};
  float64 q_roll[4]  = {cr, 0, sr, 0};
  float64 tmp[4];
  quat_mul(q_yaw, q_pitch, tmp);
  quat_mul(tmp, q_roll, q_sp);

  // attitude error: q_err = q_sp * conj(q)
  float64 q_conj[4]; quat_conjugate(ekfState.q, q_conj);
  float64 q_err[4];  quat_mul(q_sp, q_conj, q_err);

  // turn into axis-angle ~ small angle error vector
  float64 axis[3], angle;
  quat_to_axis_angle(q_err, axis, angle);
  float64 e_roll  = axis[1] * angle;  // body-y
  float64 e_pitch = axis[0] * angle;  // body-x
  float64 e_yaw   = axis[2] * angle;  // body-z

  body_rate_sp[0] = m_Params.kp_att_roll  * e_roll;
  body_rate_sp[1] = m_Params.kp_att_pitch * e_pitch;
  body_rate_sp[2] = yaw_rate_sp + m_Params.kp_att_yaw * e_yaw;

  // limit rates to RC maxima to stay safe
  body_rate_sp[0] = constrain(body_rate_sp[0], -m_Params.rc_max_rate[0], m_Params.rc_max_rate[0]);
  body_rate_sp[1] = constrain(body_rate_sp[1], -m_Params.rc_max_rate[1], m_Params.rc_max_rate[1]);
  body_rate_sp[2] = constrain(body_rate_sp[2], -m_Params.rc_max_rate[2], m_Params.rc_max_rate[2]);
}

void C_PIDController::rate_inner(const float64 body_rate_sp[3], const float64 gyro[3], float64 dt, float64 torque_cmd[3]) {
  m_State.motors_saturated = false; // reset will be set later

  for (uint8 i=0;i<3;i++) {
    float64 err = body_rate_sp[i] - gyro[i];
    // PI + D on measurement (implicit with gyro as measurement derivative already)
    m_State.i_rate[i] += m_Params.ki_rate[i] * err * dt;
    m_State.i_rate[i] = constrain(m_State.i_rate[i], -m_Params.rate_i_lim[i], m_Params.rate_i_lim[i]);
    float64 u = m_Params.kp_rate[i]*err + m_State.i_rate[i] - m_Params.kd_rate[i]*0.0; // D is tiny; can add filter if you want
    torque_cmd[i] = constrain(u, -1.0, 1.0);
  }
}

void C_PIDController::mix_and_output(float64 thrust_cmd, const float64 torque_cmd[3], bool isArmed) {
  // Quad-X mixing (FR, RR, RL, FL) - M1..M4
  // Signs assume: M1 (FR) CCW, M2 (RR) CW, M3 (RL) CCW, M4 (FL) CW
  const float64 r = m_Params.mix_roll, p = m_Params.mix_pitch, y = m_Params.mix_yaw;

  float64 u1 = thrust_cmd + r* torque_cmd[0] + p* torque_cmd[1] - y* torque_cmd[2]; // FR
  float64 u2 = thrust_cmd + r* torque_cmd[0] - p* torque_cmd[1] + y* torque_cmd[2]; // RR
  float64 u3 = thrust_cmd - r* torque_cmd[0] - p* torque_cmd[1] - y* torque_cmd[2]; // RL
  float64 u4 = thrust_cmd - r* torque_cmd[0] + p* torque_cmd[1] + y* torque_cmd[2]; // FL

  // Saturation and anti‑windup hint
  float64 maxu = std::max(std::max(u1,u2), std::max(u3,u4));
  float64 minu = std::min(std::min(u1,u2), std::min(u3,u4));
  if (maxu > m_Params.max_thrust || minu < m_Params.min_thrust) {
    m_State.motors_saturated = true;
    // scale towards center to keep all within bounds
    float64 center = thrust_cmd;
    float64 hi = m_Params.max_thrust - center;
    float64 lo = center - m_Params.min_thrust;
    float64 scale = 1.0;
    if (maxu - center > hi)  scale = std::min(scale, hi / (maxu - center));
    if (center - minu > lo)  scale = std::min(scale, lo / (center - minu));
    float64 du1 = u1 - center, du2 = u2 - center, du3 = u3 - center, du4 = u4 - center;
    u1 = center + du1*scale; u2 = center + du2*scale; u3 = center + du3*scale; u4 = center + du4*scale;
  }

  // final clamp
  u1 = constrain(u1, m_Params.min_thrust, m_Params.max_thrust);
  u2 = constrain(u2, m_Params.min_thrust, m_Params.max_thrust);
  u3 = constrain(u3, m_Params.min_thrust, m_Params.max_thrust);
  u4 = constrain(u4, m_Params.min_thrust, m_Params.max_thrust);

  // When disarmed - stop motors
  if (!isArmed) {
    u1 = u2 = u3 = u4 = 0.0;
  }

  // map [0..1] - microseconds
  auto to_us = [&](float64 u)->uint16 {
    float64 x = constrain(u, 0.0, 1.0);
    return m_Params.pwm_min_us + (uint16)((m_Params.pwm_max_us - m_Params.pwm_min_us) * x + 0.5);
  };
  //                       FR         RR         RL         FL
  PWMgen::outputWrapper(to_us(u1), to_us(u2), to_us(u3), to_us(u4));
}

} // namespace ctrl
