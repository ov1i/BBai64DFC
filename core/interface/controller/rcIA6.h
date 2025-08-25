#ifndef DFC_RC_H
#define DFC_RC_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include "dfc_types.h"
#include "utils.h"

extern "C" {
#include <ti/csl/soc.h>
}

struct CSL_ecapRegs;


namespace rc {

class C_RcIA6 {
public:
  C_RcIA6() = default;

  bool init(const DFC_t_RcParams &cfg = DFC_t_RcParams());
  void update();
  DFC_t_RcInputs getData() const { return m_Data; }

private:
  static inline float64 clampf(float64 x, float64 lo, float64 hi) {
    return x < lo ? lo : (x > hi ? hi : x);
  }

  static inline float64 fmap(float64 x, float64 i0, float64 i1, float64 o0, float64 o1) {
    float64 t = (x - i0) / (i1 - i0);
    t = clampf(t, 0.0, 1.0);
    return o0 + t * (o1 - o0);
  }

  static inline float64 pwm_to_sym(float64 us, float64 cen, float64 half) {
    return clampf((us - cen) / half, -1.0, +1.0);
  }

  static inline float64 apply_deadband(float64 x, float64 db) {
    float64 ax = std::fabs(x);
    if (ax <= db)
      return 0.0;
    float64 y = (ax - db) / (1.0 - db);
    return (x < 0.0 ? -y : y);
  }

  static inline float64 lpf(float64 prev, float64 cur, float64 a) {
    return prev + a * (cur - prev);
  }

  static sint32 preInit(uint16 id);
  static void initCaptureEvents(volatile CSL_ecapRegs *ecap);
  static bool read(volatile CSL_ecapRegs *ecap, uint32 &cap1, uint32 &cap2);

  DFC_t_RcParams m_Params{};
  DFC_t_RcInputs m_Data{};

  volatile uint32 m_Throttle_ms{0}, m_Roll_ms{0}, m_Pitch_ms{0};
  volatile uint32 m_Throttle_cap1{0}, m_Throttle_cap2{0};
  volatile uint32 m_Roll_cap1{0}, m_Roll_cap2{0};
  volatile uint32 m_Pitch_cap1{0}, m_Pitch_cap2{0};

  // filtered normalized channels
  float64 m_NormThrottle{0.0};
  float64 m_NormRoll{0.0};
  float64 m_NormlPitch{0.0};

  // arming gesture state
  uint32 m_ArmHold_ms{0};
  
  // mode gesture state
  uint32 m_ModeHold_ms{0};

  // step() timing
  uint32 m_LastStep_ms{0};
};

} // namespace rc

#endif // DFC_RC_H