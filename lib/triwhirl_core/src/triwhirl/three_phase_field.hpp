#pragma once

namespace triwhirl {

struct PhaseVoltages {
  float a;
  float b;
  float c;
};

// Produces balanced, center-biased sinusoidal phase commands for low-voltage
// motor bring-up. `amplitude_v` is the alpha/beta vector amplitude.
//
// This is intentionally an electrical-field generator, not closed-loop FOC.
// It lets us validate the 3-PWM bridge and measure pole-pair count without
// assuming the motor pole count in firmware.
PhaseVoltages makeRotatingField(float electrical_angle_rad,
                                float amplitude_v,
                                float voltage_limit_v);

}  // namespace triwhirl
