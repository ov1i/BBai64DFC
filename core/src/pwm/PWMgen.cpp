#include "motor_pwm.h"
#include <ti/drv/uart/UART_stdio.h>

// NOTE: This file is the only place you need to bind to hardware.
// If you already have an EPWM driver, call it from write_us().
// The stub below just prints once per second if pulse changes,
// so you can bring up the controller before wiring real PWM.

namespace motor_pwm {

static int s_min = 1000, s_max = 2000;
static int last_us[4] = {0,0,0,0};

void init(int pwm_min_us, int pwm_max_us) {
  s_min = pwm_min_us;
  s_max = pwm_max_us;
  UART_printf("[PWM] init: range %d..%d us\r\n", s_min, s_max);

  // TODO: configure your EPWM timers here (period for 400 Hz or 50 Hz, etc.)
  // Example pseudocode:
  // epwm_config(period_ticks, sysclk_hz);
  // epwm_set_duty(ch, ticks_for_us);
}

void write_us(int ch, int us) {
  if (ch < 0 || ch > 3) return;
  if (us < 0) us = 0;
  // TODO: write to hardware. Replace the printf with your driver call.
  if (us != last_us[ch]) {
    last_us[ch] = us;
    UART_printf("[PWM] ch%d = %d us\r\n", ch, us);
  }

  // Example:
  // epwm_set_compare(ch, ticks_for_us(us));
}

void disarm_all() {
  for (int i=0;i<4;i++) write_us(i, 0);
}

} // namespace motor_pwm
