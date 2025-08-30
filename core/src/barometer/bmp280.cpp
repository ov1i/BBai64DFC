#include "barometer/bmp280.h"
#include "i2c/i2c_helper.h"
#include <cmath>
#include <cstring>
#include <dfc_types.h>
#include <utils.h>

extern "C" {
#include <FreeRTOS.h>
#include <ti/csl/csl_types.h>
#include <ti/csl/csl_utils.h>
#include <ti/drv/i2c/i2c.h>
#include <ti/drv/i2c/soc/i2c_soc.h>
#include <ti/osal/TaskP.h>

}

namespace baro {

static inline uint64 now_ns() {
  return (uint64)TimerP_getTimeInUsecs() * 1000ull;
}

C_BMP280::C_BMP280() { std::memset(this, 0, sizeof(*this)); }

bool C_BMP280::init() {
  if (!m_i2cState) {
    m_i2cState = m_i2cHandler.init(4, I2C_400kHz);
    if (!m_i2cState) {
      return false;
    }
  }

  uint8 temp[2] = {BMP280_RST_REG, 0xB6};
  m_i2cHandler.writeRegSingletVal(BMP280_ADDRESS, temp);

  if (!updateInternalCalibReg()) {
      DebugP_log0("[BMP280] Calib read FAILED\r\n");
    return false;
  }

  // 0x10h : t_sb=0.5ms(000), filter=16(100), spi3w=0
  temp[0] = BMP280_CONFIG_REG;
  temp[1] = 0x10;
  if (!m_i2cHandler.writeRegSingletVal(BMP280_ADDRESS, temp)) {
      DebugP_log0("[BMP280] Config setting FAILED\r\n");
    return false;
  }

  // 0x33h : osrs_t=1(1), osrs_p=4(100), mode=normal(11)
  temp[0] = BMP280_CTRL_MES_REG;
  temp[1] = 0x33;
  if (!m_i2cHandler.writeRegSingletVal(BMP280_ADDRESS, temp)) {
      DebugP_log0("[BMP280] Setting of the ctrl_meas FAILED\r\n");
    return false;
  }

    DebugP_log0("[BMP280] Init OK");
  return true;
}

bool C_BMP280::update() {
  uint8 status;
  uint8 reg = BMP280_STATUS_REGISTER;
  if (!m_i2cHandler.rw(BMP280_ADDRESS, &reg, 1, &status, 1)) {
      DebugP_log0("Failed to read BMP status register\r\n");
    return false;
  }
  
  if (status & 0x8) {
      DebugP_log0("BMP data not ready\r\n");
    return false;
  }

  uint8 tempBuffer[6];
  reg = BMP280_PMSB_DATA_REGISTER;
  if (!m_i2cHandler.rw(BMP280_ADDRESS, &reg, 1, tempBuffer, 6)) {
      DebugP_log0("[BMP280] read FAILED\r\n");
    return false;
  }
  sint32 rawPressure = (sint32)((tempBuffer[0] << 12) | (tempBuffer[1] << 4) | (tempBuffer[2] >> 4));
  sint32 rawTemp = (sint32)((tempBuffer[3] << 12) | (tempBuffer[4] << 4) | (tempBuffer[5] >> 4));
  internalValCompensation(rawTemp, rawPressure);
  computeAltitude();
  m_data.ts_ns = now_ns();

  return true;
}

bool C_BMP280::updateInternalCalibReg() {
  uint8 tempBuffer[24];
  uint8 reg = BMP280_T1_CALIB_ADDR;
  if (!m_i2cHandler.rw(BMP280_ADDRESS, &reg, 1, tempBuffer, sizeof(tempBuffer))) return false;

  m_data.calibration_data.dig_T1 = (uint16)(tempBuffer[1] << 8 | tempBuffer[0]);
  m_data.calibration_data.dig_T2 = (sint16)(tempBuffer[3] << 8 | tempBuffer[2]);
  m_data.calibration_data.dig_T3 = (sint16)(tempBuffer[5] << 8 | tempBuffer[4]);
  m_data.calibration_data.dig_P1 = (uint16)(tempBuffer[7] << 8 | tempBuffer[6]);
  m_data.calibration_data.dig_P2 = (sint16)(tempBuffer[9] << 8 | tempBuffer[8]);
  m_data.calibration_data.dig_P3 = (sint16)(tempBuffer[11] << 8 | tempBuffer[10]);
  m_data.calibration_data.dig_P4 = (sint16)(tempBuffer[13] << 8 | tempBuffer[12]);
  m_data.calibration_data.dig_P5 = (sint16)(tempBuffer[15] << 8 | tempBuffer[14]);
  m_data.calibration_data.dig_P6 = (sint16)(tempBuffer[17] << 8 | tempBuffer[16]);
  m_data.calibration_data.dig_P7 = (sint16)(tempBuffer[19] << 8 | tempBuffer[18]);
  m_data.calibration_data.dig_P8 = (sint16)(tempBuffer[21] << 8 | tempBuffer[20]);
  m_data.calibration_data.dig_P9 = (sint16)(tempBuffer[23] << 8 | tempBuffer[22]);

  return true;
}

void C_BMP280::internalValCompensation(sint32 rawTempData, sint32 rawPressureData) {
  // temperature
  sint32 var1 = ((((rawTempData >> 3) - ((sint32)m_data.calibration_data.dig_T1 << 1))) * ((sint32)m_data.calibration_data.dig_T2)) >> 11;
  sint32 var2 = (((((rawTempData >> 4) - ((sint32)m_data.calibration_data.dig_T1)) * ((rawTempData >> 4) - ((sint32)m_data.calibration_data.dig_T1))) >> 12) * ((sint32)m_data.calibration_data.dig_T3)) >> 14;
  sint32 fineResolution = var1 + var2;
  sint32 processedTemp = (fineResolution * 5 + 128) >> 8; // 0.01 C
  m_data.temp = (float64)processedTemp / 100.0;

  // pressure
  sint64 var1p = ((sint64)fineResolution) - 128000;
  sint64 var2p = var1p * var1p * (sint64)m_data.calibration_data.dig_P6;
  var2p = var2p + ((var1p * (sint64)m_data.calibration_data.dig_P5) << 17);
  var2p = var2p + (((sint64)m_data.calibration_data.dig_P4) << 35);
  var1p = ((var1p * var1p * (sint64)m_data.calibration_data.dig_P3) >> 8) + ((var1p * (sint64)m_data.calibration_data.dig_P2) << 12);
  var1p = (((((sint64)1) << 47) + var1p)) * ((sint64)m_data.calibration_data.dig_P1) >> 33;
  if (var1p == 0) {
    m_data.pressure = 0.0;
    return;
  }

  sint64 processedPress = 1048576 - rawPressureData;
  processedPress = (((processedPress << 31) - var2p) * 3125) / var1p;
  var1p = (((sint64)m_data.calibration_data.dig_P9) * (processedPress >> 13) * (processedPress >> 13)) >> 25;
  var2p = (((sint64)m_data.calibration_data.dig_P8) * processedPress) >> 19;
  processedPress = ((processedPress + var1p + var2p) >> 8) + (((sint64)m_data.calibration_data.dig_P7) << 4);
  m_data.pressure = (float64)processedPress / 256.0; // Q24.8 → Pa
}

void C_BMP280::computeAltitude() {
  // h = 44330 * (1 - (P/P0)^(1/5.255))
  if (m_data.gnd.P0 <= 0) {
    m_data.altitude = 0.0;
    return;
  }
  float64 ratio = m_data.pressure / m_data.gnd.P0;
  if (ratio <= 0.0) {
    m_data.altitude = 0.0;
    return;
  }
  m_data.altitude = 44330.0 * (1.0 - std::pow(ratio, 0.1902949572));
}

} // namespace baro
