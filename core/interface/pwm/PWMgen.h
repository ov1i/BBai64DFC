#pragma once
#include <cstdint>

namespace motor_pwm {

// Call once from Controller::init()
void init(int pwm_min_us, int pwm_max_us);

// channel in [0..3], pulse width in microseconds
void write_us(int channel, int pulse_us);

// (optional) arm/disarm ESC line (e.g., set low)
void disarm_all();

} // namespace motor_pwm
