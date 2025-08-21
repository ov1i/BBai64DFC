#include "ekf/ekf.h"

namespace ekf {

inline void C_EKF::q_norm(float64 q[4]) {
  float64 n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (n > 1e-12) { q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n; }
}
inline void C_EKF::q_mul(const float64 a[4], const float64 b[4], float64 o[4]) {
  o[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  o[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  o[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  o[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}
inline void C_EKF::q_from_dtheta(const float64 dth[3], float64 q[4]) {
  float64 a = std::sqrt(dth[0]*dth[0] + dth[1]*dth[1] + dth[2]*dth[2]);
  float64 ha = 0.5 * a;
  float64 s = (a < 1e-12) ? 0.5 : std::sin(ha) / a;
  q[0] = std::cos(ha);
  q[1] = s * dth[0];
  q[2] = s * dth[1];
  q[3] = s * dth[2];
}
void C_EKF::R_from_q(const float64 q[4], float64 R[9]) {
  float64 w=q[0], x=q[1], y=q[2], z=q[3];
  float64 ww=w*w, xx=x*x, yy=y*y, zz=z*z;
  R[0] = ww + xx - yy - zz;
  R[1] = 2*(x*y - w*z);
  R[2] = 2*(x*z + w*y);
  R[3] = 2*(x*y + w*z);
  R[4] = ww - xx + yy - zz;
  R[5] = 2*(y*z - w*x);
  R[6] = 2*(x*z - w*y);
  R[7] = 2*(y*z + w*x);
  R[8] = ww - xx - yy + zz;
}

void C_EKF::reset(uint64 t0_ns) {
  std::memset(&m_state, 0, sizeof(m_state));
  m_state.q[0] = 1.0;
  m_state.t_ns = t0_ns;

  // Diagonal covariance init (use correct 15x15 stride!)
  for (uint8 i=0;i<15*15;i++) m_state.P[i]=0.0f;
  auto setD = [&](uint8 i, float64 v){ m_state.P[i*15 + i] = v; };
  setD(0,1.0); setD(1,1.0); setD(2,1.0);        // pos
  setD(3,0.5); setD(4,0.5); setD(5,0.5);        // vel
  setD(6,0.2); setD(7,0.2); setD(8,0.2);        // attitude
  setD(9,1e-3); setD(10,1e-3); setD(11,1e-3);   // gyro bias
  setD(12,1e-2); setD(13,1e-2); setD(14,1e-2);  // accel bias

  m_last_imu_ts_ns  = 0;
  m_last_mag_ts_ns  = 0;
  m_last_baro_ts_ns = 0;
}

inline void C_EKF::mat15_add_Q(float64* P, const float64* Q) {
  for (uint8 i=0;i<15;i++) P[i*15 + i] += Q[i];
}
inline void C_EKF::mat15_FP_PFT(float64* P, const float64* F, float64 dt) {
  float64 FP[15*15];
  for (uint8 i=0;i<15;i++){
    for (uint8 j=0;j<15;j++){
      float64 s=0.0;
      for (uint8 k=0;k<15;k++) s += F[i*15+k]*P[k*15+j];
      FP[i*15+j]=s;
    }
  }
  for (uint8 i=0;i<15;i++){
    for (uint8 j=0;j<15;j++){
      P[i*15+j] += dt*(FP[i*15+j] + FP[j*15+i]);
    }
  }
}
inline void C_EKF::matMN_vec(const float64* M, const float64* v, float64* out, uint8 rows, uint8 cols) {
  for (uint8 i=0;i<rows;i++){
    float64 s=0.0;
    for (uint8 j=0;j<cols;j++) s += M[i*cols + j]*v[j];
    out[i]=s;
  }
}
inline void C_EKF::axpy(float64* x, const float64* a, float64 k, uint8 n) {
  for (uint8 i=0;i<n;i++) x[i] += k*a[i];
}

void C_EKF::predict(const float64 gyro[3], const float64 acc[3], float64 dt) {
  // Guard dt
  if (!(dt > 0.0)) dt = 0.005;
  if (dt > 0.05)   dt = 0.05;

  // Bias corrected signals
  float64 w[3] = { gyro[0] - m_state.bg[0],
                  gyro[1] - m_state.bg[1],
                  gyro[2] - m_state.bg[2] };
  float64 f[3] = { acc[0]  - m_state.ba[0],
                  acc[1]  - m_state.ba[1],
                  acc[2]  - m_state.ba[2] };

  // Quaternion integration
  float64 dth[3] = { w[0]*dt, w[1]*dt, w[2]*dt };
  float64 dq[4]; q_from_dtheta(dth, dq);
  float64 qn[4]; q_mul(m_state.q, dq, qn); q_norm(qn);

  // Acc in NED (z down) and subtract gravity
  float64 Rbe[9]; R_from_q(qn, Rbe);
  float64 aN[3] = {
    Rbe[0]*f[0] + Rbe[1]*f[1] + Rbe[2]*f[2],
    Rbe[3]*f[0] + Rbe[4]*f[1] + Rbe[5]*f[2],
    Rbe[6]*f[0] + Rbe[7]*f[1] + Rbe[8]*f[2] - GRAVITY
  };

  // Nominal integration
  m_state.p[0] += m_state.v[0]*dt + 0.5*aN[0]*dt*dt;
  m_state.p[1] += m_state.v[1]*dt + 0.5*aN[1]*dt*dt;
  m_state.p[2] += m_state.v[2]*dt + 0.5*aN[2]*dt*dt;
  m_state.v[0] += aN[0]*dt;
  m_state.v[1] += aN[1]*dt;
  m_state.v[2] += aN[2]*dt;
  m_state.q[0]=qn[0]; m_state.q[1]=qn[1]; m_state.q[2]=qn[2]; m_state.q[3]=qn[3];

  // Linearized error dynamics F (15x15)
  float64 F[15*15]; std::memset(F, 0, sizeof(F));
  // dpdot = dv
  F[0*15 + 3] = 1.0; F[1*15 + 4] = 1.0; F[2*15 + 5] = 1.0;

  // dv wrt dtheta: -R*[f]_x
  float64 fx=f[0], fy=f[1], fz=f[2];
  float64 C1[9] = { 0, -fz,  fy,
                   fz, 0,  -fx,
                  -fy,  fx,  0 };
  float64 M[9] = {
    -(Rbe[0]*C1[0] + Rbe[1]*C1[3] + Rbe[2]*C1[6]),
    -(Rbe[0]*C1[1] + Rbe[1]*C1[4] + Rbe[2]*C1[7]),
    -(Rbe[0]*C1[2] + Rbe[1]*C1[5] + Rbe[2]*C1[8]),
    -(Rbe[3]*C1[0] + Rbe[4]*C1[3] + Rbe[5]*C1[6]),
    -(Rbe[3]*C1[1] + Rbe[4]*C1[4] + Rbe[5]*C1[7]),
    -(Rbe[3]*C1[2] + Rbe[4]*C1[5] + Rbe[5]*C1[8]),
    -(Rbe[6]*C1[0] + Rbe[7]*C1[3] + Rbe[8]*C1[6]),
    -(Rbe[6]*C1[1] + Rbe[7]*C1[4] + Rbe[8]*C1[7]),
    -(Rbe[6]*C1[2] + Rbe[7]*C1[5] + Rbe[8]*C1[8]),
  };
  F[3*15+6]=(float64)M[0]; F[3*15+7]=(float64)M[1]; F[3*15+8]=(float64)M[2];
  F[4*15+6]=(float64)M[3]; F[4*15+7]=(float64)M[4]; F[4*15+8]=(float64)M[5];
  F[5*15+6]=(float64)M[6]; F[5*15+7]=(float64)M[7]; F[5*15+8]=(float64)M[8];

  // dv wrt dba: -R
  F[3*15+12]=-(float64)Rbe[0]; F[3*15+13]=-(float64)Rbe[1]; F[3*15+14]=-(float64)Rbe[2];
  F[4*15+12]=-(float64)Rbe[3]; F[4*15+13]=-(float64)Rbe[4]; F[4*15+14]=-(float64)Rbe[5];
  F[5*15+12]=-(float64)Rbe[6]; F[5*15+13]=-(float64)Rbe[7]; F[5*15+14]=-(float64)Rbe[8];

  // dthetadot wrt dbg: -I  (simple bias coupling)
  F[6*15+9] = -1.0; F[7*15+10] = -1.0; F[8*15+11] = -1.0;

  // Discrete process noise (very simplified diag mapping)
  float64 Qd[15] = {0};
  float64 sg = m_params.gyro_noise;
  float64 sa = m_params.acc_noise;
  float64 sbg = m_params.gyro_bias_rw;
  float64 sba = m_params.acc_bias_rw;
  float64 dtf = (float64)dt;

  Qd[0]=1e-6f; Qd[1]=1e-6f; Qd[2]=1e-6f;  // pos
  float64 dvv = (sa*dtf)*(sa*dtf);
  Qd[3]=dvv; Qd[4]=dvv; Qd[5]=dvv;        // vel
  float64 dtt = (sg*dtf)*(sg*dtf);
  Qd[6]=dtt; Qd[7]=dtt; Qd[8]=dtt;        // att
  float64 dbg = (sbg*dtf)*(sbg*dtf);
  Qd[9]=dbg; Qd[10]=dbg; Qd[11]=dbg;      // gyro bias
  float64 dba = (sba*dtf)*(sba*dtf);
  Qd[12]=dba; Qd[13]=dba; Qd[14]=dba;     // accel bias

  mat15_FP_PFT(m_state.P, F, dtf);
  mat15_add_Q(m_state.P, Qd);
  pin_P();

  m_state.t_ns += (uint64)llround(dt * 1e9);
}

void C_EKF::inject_error(const float64 dx[15]) {
  // [dp dv dθ dbg dba]
  m_state.p[0]+=dx[0]; m_state.p[1]+=dx[1]; m_state.p[2]+=dx[2];
  m_state.v[0]+=dx[3]; m_state.v[1]+=dx[4]; m_state.v[2]+=dx[5];

  float64 dth[3] = { dx[6], dx[7], dx[8] };
  float64 dq[4]; q_from_dtheta(dth, dq);
  float64 qn[4]; q_mul(dq, m_state.q, qn); q_norm(qn);
  m_state.q[0]=qn[0]; m_state.q[1]=qn[1]; m_state.q[2]=qn[2]; m_state.q[3]=qn[3];

  m_state.bg[0]+=dx[9];  m_state.bg[1]+=dx[10]; m_state.bg[2]+=dx[11];
  m_state.ba[0]+=dx[12]; m_state.ba[1]+=dx[13]; m_state.ba[2]+=dx[14];
}
void C_EKF::pin_P() {
  for (uint8 i=0;i<15;i++){
    float64 &d = m_state.P[i*15 + i];
    if (d < 1e-9f) d = 1e-9f;
    if (d > 1e6f) d = 1e6f;
  }
}

void C_EKF::update_baro(float64 z_meas, float64 std_override) {
  // h(x) = p_z
  float64 H[15]={0}; H[2]=1.0;
  float64 R   = (std_override > 0.0 ? std_override : m_params.baro_std);
  R = R*R;

  float64 S = m_state.P[2*15 + 2] + R;

  float64 K[15];
  for (uint8 i=0;i<15;i++) K[i] = m_state.P[i*15 + 2] / S;

  float64 r = (float64)(z_meas - m_state.p[2]);

  float64 dx[15]={0};
  for (uint8 i=0;i<15;i++) dx[i] = K[i]*r;
  inject_error(dx);

  // (I - KH)P for scalar H with only index 2 nonzero → row 2 of P
  for (uint8 i=0;i<15;i++){
    for (uint8 j=0;j<15;j++){
      m_state.P[i*15 + j] -= K[i]*H[j]*m_state.P[2*15 + j];
    }
  }
  pin_P();
}

void C_EKF::update_mag_body(const float64 b_meas[3]) {
  // Predict body magnetic vector from world-frame mN
  float64 R[9]; R_from_q(m_state.q, R);
  float64 mN[3] = { m_params.magN[0], m_params.magN[1], m_params.magN[2] };
  // body = R^T * mN
  float64 b_pred[3] = {
    (float64)(R[0]*mN[0] + R[3]*mN[1] + R[6]*mN[2]),
    (float64)(R[1]*mN[0] + R[4]*mN[1] + R[7]*mN[2]),
    (float64)(R[2]*mN[0] + R[5]*mN[1] + R[8]*mN[2]),
  };
  // residual
  float64 r[3] = { (float64)(b_meas[0] - b_pred[0]),
                 (float64)(b_meas[1] - b_pred[1]),
                 (float64)(b_meas[2] - b_pred[2]) };

  // Jacobian w.r.t attitude error: ∂b/∂dθ ≈ -[b_pred]_x
  float64 bx=b_pred[0], by=b_pred[1], bz=b_pred[2];
  float64 J[9] = {    0, -bz,  by,
                  bz,   0, -bx,
                 -by,  bx,   0 };
  for (uint8 i=0;i<9;i++) J[i] = -J[i];

  // Use θθ block only (3x3) for speed
  float64 Ptt[9] = {
    m_state.P[6*15+6], m_state.P[6*15+7], m_state.P[6*15+8],
    m_state.P[7*15+6], m_state.P[7*15+7], m_state.P[7*15+8],
    m_state.P[8*15+6], m_state.P[8*15+7], m_state.P[8*15+8],
  };
  float64 JP[9];
  for (uint8 i=0;i<3;i++) for (uint8 j=0;j<3;j++){
    float64 s=0.0; for (uint8 k=0;k<3;k++) s += J[i*3+k]*Ptt[k*3+j];
    JP[i*3+j]=s;
  }
  float64 S[9];
  for (uint8 i=0;i<3;i++) for (uint8 j=0;j<3;j++){
    float64 s=0.0; for (uint8 k=0;k<3;k++) s += JP[i*3+k]*J[j*3+k];
    S[i*3+j] = s + (i==j ? m_params.mag_std*m_params.mag_std : 0.0);
  }
  // invert S (3x3)
  float64 a=S[0], b=S[1], c=S[2], d=S[3], e=S[4], f=S[5], g=S[6], h=S[7], k=S[8];
  float64 det = a*(e*k - f*h) - b*(d*k - f*g) + c*(d*h - e*g);
  if (std::fabs(det) < 1e-9f) return;
  float64 invS[9] = {
    (e*k - f*h)/det, (c*h - b*k)/det, (b*f - c&e)/det,
    (f*g - d*k)/det, (a*k - c*g)/det, (c*d - a*f)/det,
    (d*h - e*g)/det, (b*g - a*h)/det, (a*e - b*d)/det
  };

  // Kθ = Ptt * J^T * invS
  float64 JT[9] = { J[0],J[3],J[6], J[1],J[4],J[7], J[2],J[5],J[8] };
  float64 PJt[9];
  for (uint8 i=0;i<3;i++) for (uint8 j=0;j<3;j++){
    float64 s=0.0; for (uint8 m=0;m<3;m++) s += Ptt[i*3+m]*JT[m*3+j];
    PJt[i*3+j]=s;
  }
  float64 Kt[9];
  for (uint8 i=0;i<3;i++) for (uint8 j=0;j<3;j++){
    float64 s=0.0; for (uint8 m=0;m<3;m++) s += PJt[i*3+m]*invS[m*3+j];
    Kt[i*3+j]=s;
  }

  // dxθ = Kt * r
  float64 dth[3] = {
    Kt[0]*r[0] + Kt[1]*r[1] + Kt[2]*r[2],
    Kt[3]*r[0] + Kt[4]*r[1] + Kt[5]*r[2],
    Kt[6]*r[0] + Kt[7]*r[1] + Kt[8]*r[2],
  };
  float64 dx[15]={0};
  dx[6]=dth[0]; dx[7]=dth[1]; dx[8]=dth[2];
  inject_error(dx);

  // Pθθ = (I - Kθ J) Pθθ
  float64 I3[9]={1,0,0, 0,1,0, 0,0,1};
  float64 KJ[9];
  for (uint8 i=0;i<3;i++) for (uint8 j=0;j<3;j++){
    float64 s=0.0; for (uint8 m=0;m<3;m++) s += Kt[i*3+m]*J[m*3+j];
    KJ[i*3+j]=s;
  }
  float64 IKJ[9];
  for (uint8 i=0;i<3;i++) for (uint8 j=0;j<3;j++) IKJ[i*3+j]=I3[i*3+j]-KJ[i*3+j];
  float64 newP[9];
  for (uint8 i=0;i<3;i++) for (uint8 j=0;j<3;j++){
    float64 s=0.0; for (uint8 m=0;m<3;m++) s += IKJ[i*3+m]*Ptt[m*3+j];
    newP[i*3+j]=s;
  }
  m_state.P[6*15+6]=newP[0]; m_state.P[6*15+7]=newP[1]; m_state.P[6*15+8]=newP[2];
  m_state.P[7*15+6]=newP[3]; m_state.P[7*15+7]=newP[4]; m_state.P[7*15+8]=newP[5];
  m_state.P[8*15+6]=newP[6]; m_state.P[8*15+7]=newP[7]; m_state.P[8*15+8]=newP[8];
  pin_P();
}

void C_EKF::update_flow_pxrate_derot(const float64 px_rate[2], float64 fx, float64 fy,
                                     float64 height_m, float64 quality) {
  if (quality < 0.05 || height_m < 0.05) return;

  float64 vbx = -height_m * (px_rate[0] / fx);
  float64 vby = -height_m * (px_rate[1] / fy);

  float64 R[9]; R_from_q(m_state.q, R);
  float64 vN = (float64)(R[0]*vbx + R[1]*vby);
  float64 vE = (float64)(R[3]*vbx + R[4]*vby);

  float64 rN = vN - (float64)m_state.v[0];
  float64 rE = vE - (float64)m_state.v[1];

  float64 base = m_params.flow_vel_std;
  float64 std  = base * (1.0f + 0.4f*(float64)std::fmax(0.0, height_m - 1.0));
  float64 Rm   = std*std;

  float64 Pvx = m_state.P[3*15+3];
  float64 Kx  = Pvx / (Pvx + Rm);
  m_state.v[0] += Kx * rN;
  m_state.P[3*15+3] *= (1.0f - Kx);

  float64 Pvy = m_state.P[4*15+4];
  float64 Ky  = Pvy / (Pvy + Rm);
  m_state.v[1] += Ky * rE;
  m_state.P[4*15+4] *= (1.0f - Ky);

  pin_P();
}

void C_EKF::handle_imu(const DFC_t_MPU9250_Data& input) {
  if (m_last_imu_ts_ns == 0) {
    m_last_imu_ts_ns = input.ts_ns;
    if (m_state.t_ns == 0) m_state.t_ns = input.ts_ns;
    return;
  }
  float64 dt = (float64)( (input.ts_ns > m_last_imu_ts_ns)
                          ? (input.ts_ns - m_last_imu_ts_ns)
                          : 0 ) * 1e-9;

  const float64 g[3] = { input.gx, input.gy, input.gz };
  const float64 a[3] = { input.ax, input.ay, input.az };

  predict(g, a, dt);
  m_last_imu_ts_ns = input.ts_ns;

  handle_mag_if_ready(input);
}

void C_EKF::handle_mag_if_ready(const DFC_t_MPU9250_Data& input) {
  if (!input.mag_rdy) return;
  if (input.tsmag_ns == 0 || input.tsmag_ns == m_last_mag_ts_ns) return;
  // Reject very stale samples (>100 ms older than current state time)
  if (m_state.t_ns > input.tsmag_ns && (m_state.t_ns - input.tsmag_ns) > 100000000ULL) return;

  // Normalize to unit vector for attitude-only update
  float64 n = std::sqrt(input.mx * input.mx + input.my * input.my + input.mz * input.mz);
  if (n < 1e-6) return;
  float64 norm_mag[3] = { input.mx/n, input.my/n, input.mz/n };

  update_mag_body(norm_mag);
  m_last_mag_ts_ns = input.tsmag_ns;
}

void C_EKF::handle_baro(const DFC_t_BMP280_Data& input) {
  if (input.ts_ns == 0 || input.altitude == 0.0 || input.ts_ns == m_last_baro_ts_ns) return;

  // Convert to NED z (down +), is relative to home (GND)
  update_baro(input.altitude);
  m_last_baro_ts_ns = input.ts_ns;
}

} // namespace ekf
