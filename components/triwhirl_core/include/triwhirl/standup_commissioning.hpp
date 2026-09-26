#pragma once

#include "triwhirl/standup_controller.hpp"

namespace triwhirl {

// One source of truth for the standup commissioning controller used by both the
// ESP32 runtime and the native SITL. Keep experiment-specific overrides here so
// simulator/controller parity cannot drift silently.
inline StandupControllerConfig makeStandupCommissioningConfig(
    const float theta_reference_rad, const float vq_limit_v = 4.0F) {
  StandupControllerConfig config{};
  config.theta_reference_rad = theta_reference_rad;
  config.vq_limit_v = vq_limit_v;

  // Keep the proven first-approach law untouched. The values below apply only
  // after the settling latch and are the current simulation-gated commissioning
  // candidate. They are intentionally shared with firmware even though hardware
  // use remains blocked until the long-run WebUI/SITL gate is accepted.
  config.lqr_k_angle_settling = -22.33F;
  config.lqr_k_wheel_settling = 9.20F;
  config.velocity_p_recovery_unstable = 0.035F;
  config.velocity_i_recovery_unstable = 0.0F;
  config.recovery_rate_damping_v_per_rad_s = 4.31F;
  config.recovery_rate_damping_limit_v = vq_limit_v;
  config.velocity_target_limit_rad_s = 80.0F;
  return config;
}

}  // namespace triwhirl
