#ifndef DFC_PWM_H
#define DFC_PWM_H

#include <controller/rcIA6.h>

extern "C" {
#include <FreeRTOS.h>
#include <timers.h>
#include <stdint.h>
#include <cstdint>
#include <stddef.h>
#include <ti/csl/csl.h>
#include <ti/csl/soc.h>
#include <ti/csl/cslr.h>
#include <ti/csl/cslr_epwm.h>
#include <ti/osal/TimerP.h>
}
#include <utils.h>
#include <dfc_types.h>

namespace PWMgen {

// Call once at boot
sint32 preInit(uint16 id);
bool init(const DFC_t_PWMgen_Params& parmas = DFC_t_PWMgen_Params());
void setupDividersPeriod(float64 tbclk_hz, uint32 freq_hz, uint16& clkdiv, uint16& hspdiv, uint16& tbprd, float64& counts_per_us);
void setupEPWM(volatile CSL_epwmRegs* pwm);

// Calibration
bool detectESCCalibrationGesture(rc::C_RcIA6& rc, uint32 window_ms, uint32 hold_ms);
bool detectFULLCalibrationGesture(rc::C_RcIA6& rc, uint32 window_ms, uint32 hold_ms);
void calibrateESC(uint32 maxDelay_ms = 4000U, uint32 minDelay_ms = 2000U);

// helpers general
inline uint16 us_to_counts(uint32 us);
static inline uint32 now_ms() { return (uint32)(xTaskGetTickCount() * portTICK_PERIOD_MS); }

// Update one motor’s pulse width in microseconds.
void write_motor_us(uint8 motorChannel, uint32 usec);

static inline uint32 clamp_us(uint32 val, uint32 lo, uint32 hi) { return val<lo?lo:(val>hi?hi:val); }
void outputWrapper(uint32 m0, uint32 m1, uint32 m2, uint32 m3);

} // namespace PWMgen

#endif