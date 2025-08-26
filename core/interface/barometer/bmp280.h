#ifndef BMP280_H
#define BMP280_H

#include "i2c/i2c_helper.h"

extern "C" {
  #include <stddef.h>
  #include <stdint.h>
}

#include <dfc_types.h>
#include <utils.h>

namespace baro {

class C_BMP280 {
public:
  C_BMP280();

  bool init();
  bool update();

  // simple “ground” reference: call collect on boot for ~1–2s, then finalize
  inline void ground_reset() {
    m_data.gnd.P0 = 0.0;
    m_data.gnd.P0_sum = 0.0;
    m_data.gnd.P0_count = 0;
  }
  inline void ground_collect() {
    if (m_data.pressure > 0) {
      m_data.gnd.P0_sum += m_data.pressure;
      m_data.gnd.P0_count++;
    }
  }
  inline bool ground_finalize() {
    if (m_data.gnd.P0_count < 8) {
      return false;
    }
    m_data.gnd.P0 = m_data.gnd.P0_sum / m_data.gnd.P0_count;
    return true;
  }

  const DFC_t_BMP280_Data& getData() const { return m_data; }

private:
  bool updateInternalCalibReg();
  void internalValCompensation();
  void computeAltitude();

  DFC_t_BMP280_Data m_data;
  i2c::C_I2C m_i2cHandler;
  bool m_i2cState;
};

} // namespace baro

#endif