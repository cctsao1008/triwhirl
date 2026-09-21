#include "triwhirl/balance_controller.hpp"

#include <algorithm>
#include <cmath>

#include "triwhirl/upright_geometry.hpp"

namespace triwhirl {

bool validBalanceControllerConfig(const BalanceControllerConfig& config) {
  const bool finite =
      std::isfinite(config.k_theta) && std::isfinite(config.k_rate) &&
      std::isfinite(config.k_wheel) &&
      std::isfinite(config.theta_reference_rad) &&
      std::isfinite(config.capture_angle_rad) &&
      std::isfinite(config.fall_angle_rad) &&
      std::isfinite(config.vq_limit_v) &&
      std::isfinite(config.wheel_rate_limit_rad_s);
  if (!finite) {
    return false;
  }

  const float gain_norm = std::fabs(config.k_theta) +
                          std::fabs(config.k_rate) +
                          std::fabs(config.k_wheel);
  return gain_norm > 1.0e-6F && config.capture_angle_rad > 0.0F &&
         config.capture_angle_rad < config.fall_angle_rad &&
         config.fall_angle_rad <= kUprightHalfPeriodRad &&
         config.vq_limit_v > 0.0F &&
         config.wheel_rate_limit_rad_s > 0.0F;
}

BalanceControllerOutput evaluateBalanceController(
    const BalanceControllerConfig& config,
    const BalanceControllerInput& input) {
  BalanceControllerOutput output{};
  if (!validBalanceControllerConfig(config) ||
      !std::isfinite(input.theta_rad) ||
      !std::isfinite(input.theta_rate_rad_s) ||
      !std::isfinite(input.wheel_rate_rad_s)) {
    return output;
  }

  output.theta_error_rad =
      periodicUprightErrorRad(input.theta_rad, config.theta_reference_rad);
  if (!std::isfinite(output.theta_error_rad)) {
    return output;
  }

  output.vq_unsaturated_v =
      -(config.k_theta * output.theta_error_rad +
        config.k_rate * input.theta_rate_rad_s +
        config.k_wheel * input.wheel_rate_rad_s);
  if (!std::isfinite(output.vq_unsaturated_v)) {
    return output;
  }

  output.vq_v = std::clamp(output.vq_unsaturated_v, -config.vq_limit_v,
                           config.vq_limit_v);
  const float abs_error = std::fabs(output.theta_error_rad);
  const float abs_wheel_rate = std::fabs(input.wheel_rate_rad_s);
  output.capture_ready =
      abs_error <= config.capture_angle_rad &&
      abs_wheel_rate <= config.wheel_rate_limit_rad_s;
  output.inside_envelope =
      abs_error <= config.fall_angle_rad &&
      abs_wheel_rate <= config.wheel_rate_limit_rad_s;
  output.valid = true;
  return output;
}

}  // namespace triwhirl
