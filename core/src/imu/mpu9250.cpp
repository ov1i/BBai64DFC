#include "imu/mpu9250.h"
#include "i2c/i2c_helper.h"
#include <cstring>
#include <dfc_types.h>
#include <utils.h>

extern "C" {
#include <ti/csl/csl_types.h>
#include <ti/csl/csl_utils.h>
#include <ti/drv/i2c/i2c.h>
#include <ti/drv/i2c/soc/i2c_soc.h>
#include <ti/drv/uart/UART.h>
#include <ti/drv/uart/UART_stdio.h>
}

namespace imu {
static inline uint64 now_ns() {
  return (uint64)TimerP_getTimeInUsecs() * 1000ull;
}

C_IMU::C_IMU() {
  memset(&m_data, 0, sizeof(m_data));
  memset(&m_i2cHandler, 0, sizeof(m_i2cHandler));
  m_i2cState = false;
}

bool C_IMU::init() {
  if (!m_i2cState) {
    m_i2cState = m_i2cHandler.init(6, I2C_400kHz);
    if (!m_i2cState) {
      return false;
    }
  }

  uint8 reg_n_data[2] = {0, 0};
  uint8 temp = 0;
  uint8 buffer[3] = {0, 0, 0};

  // Check MPU WHO_AM_I
  UART_printf("[DEBUG]: WHO_AM_I...[ CHECKING ]\r\n");
  if (!m_i2cHandler.rw(MPU9250_ADDRESS, &MPU9250_WHO_AM_I_GENERAL, 1, &temp, 1) || temp != 0x71) {
    UART_printf("[DEBUG]: WHO_AM_I... [ FAILED ]: 0x%02X\r\n", temp);
    return false;
  }
  UART_printf("[DEBUG]: WHO_AM_I...[ SUCCESS ]: 0x%02X\r\n", temp);

  // Set G+A sample rate divider = 4
  UART_printf("[DEBUG]: Clock source...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_SMPRT_DIV;
  reg_n_data[1] = 4;
  if ( ! m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 4) {
    UART_printf("[DEBUG]: Clock source...[ FAILED ]\r\n");
    return false;
  }
  UART_printf("[DEBUG]: Clock source...[ SUCCESS ]\r\n");

  // Set clock to X gyro
  UART_printf("[DEBUG]: Clock source...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_PWR_MGMNT_1;
  reg_n_data[1] = 0x01;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 1) {
    UART_printf("[DEBUG]: Clock source...[ FAILED ]\r\n");
    return false;
  }
  UART_printf("[DEBUG]: Clock source...[ SUCCESS ]\r\n");

  // Accelerometer scale = ±4g (0x08)
  UART_printf("[DEBUG]: Acc scale...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_ACC_CONFIG_1;
  reg_n_data[1] = 0x08;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 8) {
    UART_printf("[DEBUG]: Acc scale...[ FAILED ]\r\n");
    return false;
  }
  UART_printf("[DEBUG]: Acc scale...[ SUCCESS ]\r\n");

  // Gyroscope scale = ±500 dps (0x08)
  UART_printf("[DEBUG]: Gyro scale...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_GYRO_CONFIG;
  reg_n_data[1] = 0x08;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 8) {
    UART_printf("[DEBUG]: Gyro scale...[ FAILED ]\r\n");
    return false;
  }
  UART_printf("[DEBUG]: Gyro scale...[ SUCCESS ]\r\n");

  // Low-pass filter for acc (10Hz, 0x05)
  UART_printf("[DEBUG]: Acc LPF...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_ACC_CONFIG_2;
  reg_n_data[1] = 0x05;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 5) {
    UART_printf("[DEBUG]: Acc LPF...[ FAILED ]\r\n");
    return false;
  }
  UART_printf("[DEBUG]: Acc LPF...[ SUCCESS ]\r\n");

  // Low-pass filter for gyro (10Hz, 0x05)
  UART_printf("[DEBUG]: Gyro LPF...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_CONFIG;
  reg_n_data[1] = 0x05;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 5) {
    UART_printf("[DEBUG]: Gyro LPF...[ FAILED ]\r\n");
    return false;
  }
  UART_printf("[DEBUG]: Gyro LPF...[ SUCCESS ]\r\n");

  // Disable I2C master (enable bypass to AK8963 mag)
  UART_printf("[DEBUG]: I2C master disable...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_USER_CTRL;
  reg_n_data[1] = 0x00;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 0) {
    UART_printf("[DEBUG]: I2C master disable...[ FAILED ]\r\n");
    return false;
  }
  UART_printf("[DEBUG]: I2C master disable...[ SUCCESS ]\r\n");

  // Enable bypass multiplexer
  UART_printf("[DEBUG]: Bypass enable...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_INT_PIN_CONFIG;
  reg_n_data[1] = 0x02;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 2) {
    UART_printf("[DEBUG]: Bypass enable...[ FAILED ]\r\n");
    return false;
  }
  UART_printf("[DEBUG]: Bypass enable...[ SUCCESS ]\r\n");

  // --- Magnetometer (AK8963) ---
  // Check AK8963 WHO_AM_I
  UART_printf("[DEBUG]: MAG WHO_AM_I...[ CHECKING ]\r\n");
  if (!m_i2cHandler.rw(MPU9250_AK8963_ADDR, &MPU9250_AK8963_DEVICE_ID, 1, &temp,
                       1) ||
      temp != 0x48) {
    UART_printf("[DEBUG]: MAG WHO_AM_I...[ FAILED ]: 0x%02X\r\n", temp);
    return false;
  }
  UART_printf("[DEBUG]: MAG WHO_AM_I...[ SUCCESS ]: 0x%02X\r\n", temp);

  // Enter Fuse ROM access mode (for sensitivity adjustment)
  UART_printf("[DEBUG]: MAG Fuse ROM...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_MAG_CONTROL_CONFIG;
  reg_n_data[1] = 0x0F;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_AK8963_ADDR, reg_n_data)) {
    UART_printf("[DEBUG]: MAG Fuse ROM...[ FAILED ]\r\n");
    return false;
  }

  // Read sensitivity adjustment values
  UART_printf("[DEBUG]: MAG ASA read...[ SETTING ]\r\n");
  if (!m_i2cHandler.rw(MPU9250_AK8963_ADDR, &MPU9250_MAG_ASAX_CONFIG, 1, buffer, 3)) {
    UART_printf("[DEBUG]: MAG ASA read...[ FAILED ]\r\n");
    return false;
  }
  m_data.mag_adjustment[0] = ((buffer[0] - 128) * 0.5 / 128.0) + 1.0; // X-axis
  m_data.mag_adjustment[1] = ((buffer[1] - 128) * 0.5 / 128.0) + 1.0; // Y-axis
  m_data.mag_adjustment[2] = ((buffer[2] - 128) * 0.5 / 128.0) + 1.0; // Z-axis

  // Reset mag
  UART_printf("[DEBUG]: MAG reset..[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_MAG_CONTROL_CONFIG;
  reg_n_data[1] = 0x00;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_AK8963_ADDR, reg_n_data)) {
    UART_printf("[DEBUG]: MAG reset...[ FAILED ]\r\n");
    return false;
  }

  // Continuous mode 2 (100Hz, 16-bit)
  UART_printf("[DEBUG]: MAG continuous mode...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_MAG_CONTROL_CONFIG;
  reg_n_data[1] = 0x16;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_AK8963_ADDR, reg_n_data)) {
    UART_printf("[DEBUG]: MAG continuous mode...[ FAILED ]\r\n");
    return false;
  }

  UART_printf("[DEBUG]: MPU9250 init complete!\r\n");
  return true;
}

bool C_IMU::update() {
  uint8 wbuffer[1] = {MPU9250_ACC_X_H};
  uint8 rbuffer[14] = {0};

  // Read 14 bytes: accel (6), gyro (6), temp (2)
  if (!m_i2cHandler.rw(MPU9250_ADDRESS, &wbuffer[0], 1, &rbuffer[0], 14)) {
    UART_printf("Failed to read data from MPU9250\r\n");
    return false;
  }

  // Accel
  m_data.ax = (float64)((rbuffer[0] << 8) | rbuffer[1]) * acc_cvt_cst;
  m_data.ay = (float64)((rbuffer[2] << 8) | rbuffer[3]) * acc_cvt_cst;
  m_data.az = (float64)((rbuffer[4] << 8) | rbuffer[5]) * acc_cvt_cst;

  // Gyro
  m_data.gx = (float64)((rbuffer[8] << 8) | rbuffer[9]) * gyro_cvt_cst;
  m_data.gy = (float64)((rbuffer[10] << 8) | rbuffer[11]) * gyro_cvt_cst;
  m_data.gz = (float64)((rbuffer[12] << 8) | rbuffer[13]) * gyro_cvt_cst;

  // Temp
  sint16 temp_raw = (rbuffer[6] << 8) | rbuffer[7];
  m_data.temp = static_cast<float64>(temp_raw);
  m_data.ts_ns = now_ns();

  // Mag
  uint8 status1;
  if (!m_i2cHandler.rw(MPU9250_AK8963_ADDR, &MPU9250_MAG_STATUS_1, 1, &status1, 1)) {
    UART_printf("Failed to read AK8963 status1\r\n");
    return false;
  }
  if (!(status1 & 0x01)) {
    UART_printf("Magnetometer data not ready\r\n");
    m_data.mag_rdy = false; // will flag data was not changed from one reading of the IMU to the next (mag -> 100Hz others -> 200Hz)
    return true;
  }

  // Read 7 bytes: data(6), status2(1))
  uint8 mag_data[7] = {0};
  if (!m_i2cHandler.rw(MPU9250_AK8963_ADDR, &MPU9250_MAG_X_L, 1, mag_data, 7)) {
    UART_printf("Failed to read AK8963 mag data\r\n");
    return false;
  }

  if (mag_data[6] & 0x08) {
    UART_printf("Magnetometer overflow!\r\n");
    return false;
  }

  m_data.mx = (float64)((mag_data[1] << 8) | mag_data[0]) * MAG_UT_PER_LSB * m_data.mag_adjustment[0];
  m_data.my = (float64)((mag_data[3] << 8) | mag_data[2]) * MAG_UT_PER_LSB * m_data.mag_adjustment[1];
  m_data.mz = (float64)((mag_data[5] << 8) | mag_data[4]) * MAG_UT_PER_LSB * m_data.mag_adjustment[2];

  m_data.ts_ns = now_ns();
  m_data.mag_rdy = true;

  return true;
}

} // namespace imu
