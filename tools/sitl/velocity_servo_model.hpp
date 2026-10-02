#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace triwhirl::sitl {

// Provisional mechanical-domain velocity-servo surrogate for SITL only.
//
// The parameters are deliberately invalid by default. Nothing here is a
// SimpleFOC PI/LPF setting or hardware commissioning value; those properties
// must later be identified independently from the real motor path.
struct VelocityServoConfig {
  double target_velocity_limit_rad_s =
      std::numeric_limits<double>::quiet_NaN();
  double time_constant_s = std::numeric_limits<double>::quiet_NaN();
  double acceleration_limit_rad_s2 =
      std::numeric_limits<double>::quiet_NaN();
};

inline bool validVelocityServoConfig(const VelocityServoConfig& config) {
  return std::isfinite(config.target_velocity_limit_rad_s) &&
         config.target_velocity_limit_rad_s > 0.0 &&
         std::isfinite(config.time_constant_s) && config.time_constant_s > 0.0 &&
         std::isfinite(config.acceleration_limit_rad_s2) &&
         config.acceleration_limit_rad_s2 > 0.0;
}

struct VelocityServoOutput {
  bool valid = false;
  double requested_target_velocity_rad_s = 0.0;
  double applied_target_velocity_rad_s = 0.0;
  double velocity_error_rad_s = 0.0;
  double wheel_accel_command_rad_s2 = 0.0;
  bool target_saturated = false;
  bool acceleration_saturated = false;
};

// Stateless first-order closed-loop surrogate:
//
//   wheel_accel = clamp((target_velocity - wheel_velocity) / tau,
//                       -acceleration_limit, +acceleration_limit)
//
// It exists to make the target-velocity -> mechanical-response boundary
// dimensionally explicit before any fuzzy controller is tuned against SITL.
class VelocityServoModel {
 public:
  explicit VelocityServoModel(const VelocityServoConfig& config)
      : config_(config) {}

  bool valid() const { return validVelocityServoConfig(config_); }

  VelocityServoOutput evaluate(const double target_velocity_rad_s,
                               const double wheel_velocity_rad_s) const {
    VelocityServoOutput output{};
    if (!valid() || !std::isfinite(target_velocity_rad_s) ||
        !std::isfinite(wheel_velocity_rad_s)) {
      return output;
    }

    output.requested_target_velocity_rad_s = target_velocity_rad_s;
    output.applied_target_velocity_rad_s = std::clamp(
        target_velocity_rad_s, -config_.target_velocity_limit_rad_s,
        config_.target_velocity_limit_rad_s);
    output.target_saturated =
        output.applied_target_velocity_rad_s != target_velocity_rad_s;

    output.velocity_error_rad_s =
        output.applied_target_velocity_rad_s - wheel_velocity_rad_s;
    const double requested_acceleration_rad_s2 =
        output.velocity_error_rad_s / config_.time_constant_s;
    output.wheel_accel_command_rad_s2 = std::clamp(
        requested_acceleration_rad_s2, -config_.acceleration_limit_rad_s2,
        config_.acceleration_limit_rad_s2);
    output.acceleration_saturated =
        output.wheel_accel_command_rad_s2 != requested_acceleration_rad_s2;
    output.valid = std::isfinite(output.velocity_error_rad_s) &&
                   std::isfinite(output.wheel_accel_command_rad_s2);
    if (!output.valid) {
      return VelocityServoOutput{};
    }
    return output;
  }

 private:
  VelocityServoConfig config_{};
};

}  // namespace triwhirl::sitl
