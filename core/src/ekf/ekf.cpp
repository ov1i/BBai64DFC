#include "ekh.h"

void ekh::ekh_interface::init() {
    // Initialize quaternion to [1, 0, 0, 0] and zero gyro bias
    memset(&m_ekfParams, 0U, sizeof(DFC_t_EKF_params));
    m_ekfParams.X[0] = 1.0f;

    // Initialize covariance matrix P with small values
    for (uint8 i = 0; i < 7u; i++) {
        for (uint8 j = 0; j < 7u; j++) {
            m_ekfParams.P[i][j] = (i == j) ? 0.01f : 0.0f;
        }
    }
}

void ekh::ekh_interface::estimate(DFC_t_MPU9250_Data &imuData, float32 &dt) {
    predict(imuData, dt);
    update(imuData);
}

void ekh::ekh_interface::predict(DFC_t_MPU9250_Data &imuData, float32 &dt) {
    float64 wx = (imuData.gx - m_ekfParams.X[4]) * deg_2_rad_cst;
    float64 wy = (imuData.gy - m_ekfParams.X[5]) * deg_2_rad_cst;
    float64 wz = (imuData.gz - m_ekfParams.X[6]) * deg_2_rad_cst;

    float64 q0 = m_ekfParams.X[0], q1 = m_ekfParams.X[1], q2 = m_ekfParams.X[2], q3 = m_ekfParams.X[3];

    m_ekfParams.X[0] += 0.5 * dt * (-q1 * wx - q2 * wy - q3 * wz);
    m_ekfParams.X[1] += 0.5 * dt * (q0 * wx + q2 * wz - q3 * wy);
    m_ekfParams.X[2] += 0.5 * dt * (q0 * wy - q1 * wz + q3 * wx);
    m_ekfParams.X[3] += 0.5 * dt * (q0 * wz + q1 * wy - q2 * wx);

    normalizeQuat();
}

void ekh::ekh_interface::update(DFC_t_MPU9250_Data &imuData) {
    float64 ax = imuData.ax, ay = imuData.ay, az = imuData.az;
    float64 mx = imuData.mx, my = imuData.my, mz = imuData.mz;

    float64 acc_norm = sqrt(ax * ax + ay * ay + az * az);
    if (acc_norm > 0.0) {
        ax /= acc_norm; ay /= acc_norm; az /= acc_norm;
    }

    float64 mag_norm = sqrt(mx * mx + my * my + mz * mz);
    if (mag_norm > 0.0) {
        mx /= mag_norm; my /= mag_norm; mz /= mag_norm;
    }

    float64 v_acc[3] = {0, 0, 1};  // Gravity direction
    float64 v_mag[3] = {1, 0, 0};  // Magnetic north

    float64 e_acc[3] = {ax - v_acc[0], ay - v_acc[1], az - v_acc[2]};
    float64 e_mag[3] = {mx - v_mag[0], my - v_mag[1], mz - v_mag[2]};

    m_ekfParams.X[0] += 0.1 * (e_acc[0] + e_mag[0]);
    m_ekfParams.X[1] += 0.1 * (e_acc[1] + e_mag[1]);
    m_ekfParams.X[2] += 0.1 * (e_acc[2] + e_mag[2]);

    normalizeQuat();
}

void ekh::ekh_interface::normalizeQuat() {
    float64 *q = m_ekfParams.X;
    float64 norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > 0.0) {
        for (int i = 0; i < 4; ++i)
            q[i] /= norm;
    }
}
