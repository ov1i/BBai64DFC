#ifndef DFC_PWM_H
#define DFC_PWM_H

extern "C" {
#include <stdint.h>
#include <cstdint>
#include <stddef.h>
#include <ti/csl/csl.h>
#include <ti/csl/soc.h>
#include <ti/csl/cslr.h>
#include <ti/csl/cslr_epwm.h>
}
#include <utils.h>
#include <dfc_types.h>

namespace PWMgen {

// Call once at boot
static sint32 preInit(uint16 id);
static bool init(const DFC_t_PWMgen_Params& parmas = DFC_t_PWMgen_Params());
static void setupDividersPeriod(float64 tbclk_hz, uint32 freq_hz, uint16& clkdiv, uint16& hspdiv, uint16& tbprd, float64& counts_per_us);
static void setupEPWM(volatile CSL_epwmRegs* pwm);

// helpers
static inline uint16 us_to_counts(uint32 us);

// Update one motor’s pulse width in microseconds.
static void write_motor_us(uint8 motorChannel, uint32 usec);

static inline uint32 clamp_us(uint32 val, uint32 lo, uint32 hi) { return val<lo?lo:(val>hi?hi:val); }
static void outputWrapper(uint32 m0, uint32 m1, uint32 m2, uint32 m3);

} // namespace PWMgen

#endif