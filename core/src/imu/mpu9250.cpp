#include "imu/mpu9250.h"
#include "i2c/i2c_helper.h"
#include <cstring>
#include <dfc_types.h>
#include <utils.h>

extern "C" {
#include <FreeRTOS.h>
#include <timers.h>
#include <ti/osal/TimerP.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/csl_utils.h>
#include <ti/drv/i2c/i2c.h>
#include <ti/drv/i2c/soc/i2c_soc.h>
#include <ti/osal/TaskP.h>

}

namespace imu {
static inline uint64 now_ns() { return (uint64)TimerP_getTimeInUsecs() * 1000ull; }

C_IMU::C_IMU() {
  memset(&m_data, 0, sizeof(m_data));
  memset(&m_params, 0, sizeof(m_params));
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
    DebugP_log0("[DEBUG]: WHO_AM_I...[ CHECKING ]\r\n");
  reg_n_data[0] = MPU9250_WHO_AM_I_GENERAL;
  if (!m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) || temp != 0x71) {
      DebugP_log0("[DEBUG]: WHO_AM_I... [ FAILED ]\r\n");
    return false;
  }
    DebugP_log0("[DEBUG]: WHO_AM_I...[ SUCCESS ]\r\n");

  // Set G+A sample rate divider = 4
    DebugP_log0("[DEBUG]: Clock source...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_SMPRT_DIV;
  reg_n_data[1] = 4;
  if ( ! m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 4) {
      DebugP_log0("[DEBUG]: Clock source...[ FAILED ]\r\n");
    return false;
  }
    DebugP_log0("[DEBUG]: Clock source...[ SUCCESS ]\r\n");

  // Set clock to X gyro
    DebugP_log0("[DEBUG]: Clock source...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_PWR_MGMNT_1;
  reg_n_data[1] = 0x01;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 1) {
      DebugP_log0("[DEBUG]: Clock source...[ FAILED ]\r\n");
    return false;
  }
    DebugP_log0("[DEBUG]: Clock source...[ SUCCESS ]\r\n");

  // Accelerometer scale = ±4g (0x08)
    DebugP_log0("[DEBUG]: Acc scale...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_ACC_CONFIG_1;
  reg_n_data[1] = 0x08;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 8) {
      DebugP_log0("[DEBUG]: Acc scale...[ FAILED ]\r\n");
    return false;
  }
    DebugP_log0("[DEBUG]: Acc scale...[ SUCCESS ]\r\n");

  // Gyroscope scale = ±500 dps (0x08)
    DebugP_log0("[DEBUG]: Gyro scale...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_GYRO_CONFIG;
  reg_n_data[1] = 0x08;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 8) {
      DebugP_log0("[DEBUG]: Gyro scale...[ FAILED ]\r\n");
    return false;
  }
    DebugP_log0("[DEBUG]: Gyro scale...[ SUCCESS ]\r\n");

  // Low-pass filter for acc (10Hz, 0x05)
    DebugP_log0("[DEBUG]: Acc LPF...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_ACC_CONFIG_2;
  reg_n_data[1] = 0x05;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 5) {
      DebugP_log0("[DEBUG]: Acc LPF...[ FAILED ]\r\n");
    return false;
  }
    DebugP_log0("[DEBUG]: Acc LPF...[ SUCCESS ]\r\n");

  // Low-pass filter for gyro (10Hz, 0x05)
    DebugP_log0("[DEBUG]: Gyro LPF...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_CONFIG;
  reg_n_data[1] = 0x05;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 5) {
      DebugP_log0("[DEBUG]: Gyro LPF...[ FAILED ]\r\n");
    return false;
  }
    DebugP_log0("[DEBUG]: Gyro LPF...[ SUCCESS ]\r\n");

  // Disable I2C master (enable bypass to AK8963 mag)
    DebugP_log0("[DEBUG]: I2C master disable...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_USER_CTRL;
  reg_n_data[1] = 0x00;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 0) {
      DebugP_log0("[DEBUG]: I2C master disable...[ FAILED ]\r\n");
    return false;
  }
    DebugP_log0("[DEBUG]: I2C master disable...[ SUCCESS ]\r\n");

  // Enable bypass multiplexer
    DebugP_log0("[DEBUG]: Bypass enable...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_INT_PIN_CONFIG;
  reg_n_data[1] = 0x02;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_ADDRESS, reg_n_data) ||
      !m_i2cHandler.rw(MPU9250_ADDRESS, &reg_n_data[0], 1, &temp, 1) ||
      temp != 2) {
      DebugP_log0("[DEBUG]: Bypass enable...[ FAILED ]\r\n");
    return false;
  }
    DebugP_log0("[DEBUG]: Bypass enable...[ SUCCESS ]\r\n");

  // --- Magnetometer (AK8963) ---
  // Check AK8963 WHO_AM_I
    DebugP_log0("[DEBUG]: MAG WHO_AM_I...[ CHECKING ]\r\n");
  reg_n_data[0] = MPU9250_AK8963_DEVICE_ID;
  if (!m_i2cHandler.rw(MPU9250_AK8963_ADDR, &reg_n_data[0], 1, &temp, 1) || temp != 0x48) {
      DebugP_log0("[DEBUG]: MAG WHO_AM_I...[ FAILED ]\r\n");
    return false;
  }
    DebugP_log0("[DEBUG]: MAG WHO_AM_I...[ SUCCESS ]\r\n");

  // Enter Fuse ROM access mode (for sensitivity adjustment)
    DebugP_log0("[DEBUG]: MAG Fuse ROM...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_MAG_CONTROL_CONFIG;
  reg_n_data[1] = 0x0F;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_AK8963_ADDR, reg_n_data)) {
      DebugP_log0("[DEBUG]: MAG Fuse ROM...[ FAILED ]\r\n");
    return false;
  }

  // Read sensitivity adjustment values
    DebugP_log0("[DEBUG]: MAG ASA read...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_MAG_ASAX_CONFIG;
  if (!m_i2cHandler.rw(MPU9250_AK8963_ADDR, &reg_n_data[0], 1, buffer, 3)) {
      DebugP_log0("[DEBUG]: MAG ASA read...[ FAILED ]\r\n");
    return false;
  }
  m_data.mag_adjustment[0] = ((buffer[0] - 128) * 0.5 / 128.0) + 1.0; // X-axis
  m_data.mag_adjustment[1] = ((buffer[1] - 128) * 0.5 / 128.0) + 1.0; // Y-axis
  m_data.mag_adjustment[2] = ((buffer[2] - 128) * 0.5 / 128.0) + 1.0; // Z-axis

  // Reset mag
    DebugP_log0("[DEBUG]: MAG reset..[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_MAG_CONTROL_CONFIG;
  reg_n_data[1] = 0x00;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_AK8963_ADDR, reg_n_data)) {
      DebugP_log0("[DEBUG]: MAG reset...[ FAILED ]\r\n");
    return false;
  }

  // Continuous mode 2 (100Hz, 16-bit)
    DebugP_log0("[DEBUG]: MAG continuous mode...[ SETTING ]\r\n");
  reg_n_data[0] = MPU9250_MAG_CONTROL_CONFIG;
  reg_n_data[1] = 0x16;
  if (!m_i2cHandler.writeRegSingletVal(MPU9250_AK8963_ADDR, reg_n_data)) {
      DebugP_log0("[DEBUG]: MAG continuous mode...[ FAILED ]\r\n");
    return false;
  }

    DebugP_log0("[DEBUG]: MPU9250 init complete!\r\n");
  return true;
}

bool C_IMU::update() {
  uint8 wbuffer[1] = {MPU9250_ACC_X_H};
  uint8 rbuffer[14] = {0};

  // Read 14 bytes: accel (6), gyro (6), temp (2)
  if (!m_i2cHandler.rw(MPU9250_ADDRESS, &wbuffer[0], 1, &rbuffer[0], 14)) {
      DebugP_log0("Failed to read data from MPU9250\r\n");
    return false;
  }

  // Accel
  processAcc(rbuffer);

  // Gyro
  m_data.gx = (float64)((rbuffer[8] << 8) | rbuffer[9]) * gyro_cvt_cst;
  m_data.gy = (float64)((rbuffer[10] << 8) | rbuffer[11]) * gyro_cvt_cst;
  m_data.gz = (float64)((rbuffer[12] << 8) | rbuffer[13]) * gyro_cvt_cst;

  // Temp
  m_data.temp = (float64)((sint16)((rbuffer[6]<<8)|rbuffer[7]));
  m_data.ts_ns = now_ns();

  // Mag
  uint8 status1;
  uint8 reg = MPU9250_MAG_STATUS_1;
  if (!m_i2cHandler.rw(MPU9250_AK8963_ADDR, &reg, 1, &status1, 1)) {
      DebugP_log0("Failed to read AK8963 status1\r\n");
    return false;
  }
  if (!(status1 & 0x01)) {
    m_data.mag_rdy = false; // will flag data was not changed from one reading of the IMU to the next (mag -> 100Hz others -> 200Hz)
    return true;
  }

  // Read 7 bytes: data(6), status2(1))
  uint8 mag_data[7] = {0};
  reg = MPU9250_MAG_X_L;
  if (!m_i2cHandler.rw(MPU9250_AK8963_ADDR, &reg, 1, mag_data, 7)) {
      DebugP_log0("Failed to read AK8963 mag data\r\n");
    return false;
  }

  if (mag_data[6] & 0x08) {
      DebugP_log0("Magnetometer overflow!\r\n");
    return false;
  }

  // Mag
  processMag(mag_data);

  m_data.tsmag_ns = now_ns();
  m_data.mag_rdy = true;

  return true;
}

bool C_IMU::readRawOnceForCalib(DFC_t_MPU9250_Data& data) {
  // acc+gyro+temp
  uint8 reg = MPU9250_ACC_X_H;
  uint8 buffer[14];
  if (!m_i2cHandler.rw(MPU9250_ADDRESS, &reg, 1, &buffer[0], 14)) return false;

  data.ax = (float64)((buffer[0]<<8)  | buffer[1])  * acc_cvt_cst;
  data.ay = (float64)((buffer[2]<<8)  | buffer[3])  * acc_cvt_cst;
  data.az = (float64)((buffer[4]<<8)  | buffer[5])  * acc_cvt_cst;
  data.gx = (float64)((buffer[8]<<8)  | buffer[9])  * gyro_cvt_cst;
  data.gy = (float64)((buffer[10]<<8) | buffer[11]) * gyro_cvt_cst;
  data.gz = (float64)((buffer[12]<<8) | buffer[13]) * gyro_cvt_cst;
  data.temp = (float64)((sint16)((buffer[6]<<8)|buffer[7]));
  data.ts_ns = now_ns();

  // mag
  data.mag_rdy = false;
  uint8 st1;
  reg = MPU9250_MAG_STATUS_1;

  if (m_i2cHandler.rw(MPU9250_AK8963_ADDR, &reg, 1, &st1, 1) && (st1 & 0x01)) {
    uint8 magBuffer[7]={0};
    reg = MPU9250_MAG_X_L;
    
    if (m_i2cHandler.rw(MPU9250_AK8963_ADDR, &reg, 1, magBuffer, 7) && !(magBuffer[6] & 0x08)) {
      data.mx = (float64)((magBuffer[1]<<8)|magBuffer[0]) * MAG_UT_PER_LSB * m_data.mag_adjustment[0];
      data.my = (float64)((magBuffer[3]<<8)|magBuffer[2]) * MAG_UT_PER_LSB * m_data.mag_adjustment[1];
      data.mz = (float64)((magBuffer[5]<<8)|magBuffer[4]) * MAG_UT_PER_LSB * m_data.mag_adjustment[2];
      data.mag_rdy = true;
      data.tsmag_ns = now_ns();
    }
  }
  return true;
}

bool C_IMU::getCalibrationGyroBias(uint32 window_ms, float64 *bg) {
  uint32 ts = xTaskGetTickCount();
  float64 sampleX = 0, sampleY = 0, sampleZ = 0; 
  uint32 samples = 0;

  while ((xTaskGetTickCount()-ts)*portTICK_PERIOD_MS < window_ms) {
    DFC_t_MPU9250_Data data{};
    if (readRawOnceForCalib(data)) {
      sampleX += data.gx; sampleY += data.gy; sampleZ += data.gz;
      ++samples;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  if (samples < 50) return false;

  bg[0] = sampleX/samples; 
  bg[1] = sampleY/samples; 
  bg[2] = sampleZ/samples;

  return true;
}

void C_IMU::getRawAccMagSample(float64 *accRaw, float64 *magRaw) {
  DFC_t_MPU9250_Data data{};
  if(readRawOnceForCalib(data)) {
    if(!data.mag_rdy) return;

    accRaw[0] = data.ax;
    accRaw[1] = data.ay;
    accRaw[2] = data.az;

    magRaw[0] = data.mx; 
    magRaw[1] = data.my; 
    magRaw[2] = data.mz; 
  }
}

void C_IMU::processAcc(uint8 rbuffer[14]) {
  float64 rawAccData[3] = { 0 };
  // Compensate biases
  rawAccData[0] = ((float64)((rbuffer[0] << 8) | rbuffer[1]) * acc_cvt_cst) - m_params.acc_b[0];
  rawAccData[1] = ((float64)((rbuffer[2] << 8) | rbuffer[3]) * acc_cvt_cst) - m_params.acc_b[1];
  rawAccData[2] = ((float64)((rbuffer[4] << 8) | rbuffer[5]) * acc_cvt_cst) - m_params.acc_b[2];

  // Correction based on ellipsoid map
  float64 processedData[3] = { 0 };
  mapValues(m_params.acc_M, rawAccData, processedData);

  // Meas freeze values
  m_data.ax = processedData[0];
  m_data.ay = processedData[1];
  m_data.az = processedData[2];
}

void C_IMU::processMag(uint8 mag_data[7]) {
  float64 rawMagData[3] = { 0 };
  // Compensate biases
  rawMagData[0] = ((float64)((mag_data[1] << 8) | mag_data[0]) * MAG_UT_PER_LSB * m_data.mag_adjustment[0]) - m_params.mag_b[0];
  rawMagData[1] = ((float64)((mag_data[3] << 8) | mag_data[2]) * MAG_UT_PER_LSB * m_data.mag_adjustment[1]) - m_params.mag_b[1];
  rawMagData[2] = ((float64)((mag_data[5] << 8) | mag_data[4]) * MAG_UT_PER_LSB * m_data.mag_adjustment[2]) - m_params.mag_b[2];
  // Correction based on ellipsoid map
  float64 processedData[3] = { 0 };
  mapValues(m_params.mag_M, rawMagData, processedData);

  // Meas freeze values
  m_data.mx = processedData[0];
  m_data.my = processedData[1];
  m_data.mz = processedData[2];
}

} // namespace imu
