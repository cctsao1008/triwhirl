#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "triwhirl/fuzzy.hpp"

namespace triwhirl {

// Mechanical-domain state consumed by the target full-fuzzy attitude layer.
// No electrical-motor quantity belongs in this interface.
struct FuzzyBalanceInput {
  float theta_error_rad = 0.0F;
  float theta_rate_rad_s = 0.0F;
  float wheel_velocity_rad_s = 0.0F;
};

struct FuzzyBalanceConfig {
  float theta_error_scale_rad = std::numeric_limits<float>::quiet_NaN();
  float theta_rate_scale_rad_s = std::numeric_limits<float>::quiet_NaN();
  float wheel_velocity_scale_rad_s = std::numeric_limits<float>::quiet_NaN();
  float target_velocity_limit_rad_s =
      std::numeric_limits<float>::quiet_NaN();

  // Normalized zero-order Sugeno singleton surface. Rule indexing matches
  // fuzzy::evaluateSugeno3D():
  //   (theta_term * 5 + theta_rate_term) * 5 + wheel_term
  // with linguistic order NL, NS, ZE, PS, PL.
  std::array<float, fuzzy::kFiveTermRuleCount3D> target_velocity_singletons{};

  // Deliberately separate from numeric validity: an all-zero default surface
  // must not silently become an accepted controller tune.
  bool rule_surface_configured = false;
};

inline bool validFuzzyBalanceConfig(const FuzzyBalanceConfig& config) {
  if (!std::isfinite(config.theta_error_scale_rad) ||
      !(config.theta_error_scale_rad > 0.0F) ||
      !std::isfinite(config.theta_rate_scale_rad_s) ||
      !(config.theta_rate_scale_rad_s > 0.0F) ||
      !std::isfinite(config.wheel_velocity_scale_rad_s) ||
      !(config.wheel_velocity_scale_rad_s > 0.0F) ||
      !std::isfinite(config.target_velocity_limit_rad_s) ||
      !(config.target_velocity_limit_rad_s > 0.0F) ||
      !config.rule_surface_configured) {
    return false;
  }

  for (const float singleton : config.target_velocity_singletons) {
    if (!std::isfinite(singleton) || singleton < -1.0F || singleton > 1.0F) {
      return false;
    }
  }
  return true;
}

struct FuzzyBalanceOutput {
  bool valid = false;

  // State after conversion to the controller's dimensionless universes.
  std::array<float, 3> requested_normalized_input{};
  std::array<float, 3> applied_normalized_input{};
  std::array<bool, 3> input_clamped{};

  float target_velocity_normalized = 0.0F;
  float target_velocity_rad_s = 0.0F;
  float rule_weight_sum = 0.0F;
  float max_rule_weight = 0.0F;
};

// Allocation-free near-upright fuzzy attitude-control boundary.
//
// The controller owns only the nonlinear mapping from body/wheel mechanical
// state to a bounded wheel target velocity. SimpleFOC remains responsible for
// velocity regulation, FOC and PWM in the target architecture.
class FuzzyBalanceController {
 public:
  explicit FuzzyBalanceController(const FuzzyBalanceConfig& config)
      : config_(config) {}

  bool valid() const { return validFuzzyBalanceConfig(config_); }

  FuzzyBalanceOutput evaluate(const FuzzyBalanceInput& input) const {
    FuzzyBalanceOutput output{};
    if (!valid() || !std::isfinite(input.theta_error_rad) ||
        !std::isfinite(input.theta_rate_rad_s) ||
        !std::isfinite(input.wheel_velocity_rad_s)) {
      return output;
    }

    output.requested_normalized_input = {{
        input.theta_error_rad / config_.theta_error_scale_rad,
        input.theta_rate_rad_s / config_.theta_rate_scale_rad_s,
        input.wheel_velocity_rad_s / config_.wheel_velocity_scale_rad_s,
    }};

    for (const float value : output.requested_normalized_input) {
      if (!std::isfinite(value)) return FuzzyBalanceOutput{};
    }

    const std::array<fuzzy::FiveTermMembership, 3> memberships{{
        fuzzy::fiveTermMembership(output.requested_normalized_input[0]),
        fuzzy::fiveTermMembership(output.requested_normalized_input[1]),
        fuzzy::fiveTermMembership(output.requested_normalized_input[2]),
    }};

    for (std::size_t i = 0; i < memberships.size(); ++i) {
      output.applied_normalized_input[i] = std::clamp(
          output.requested_normalized_input[i], -1.0F, 1.0F);
      output.input_clamped[i] = memberships[i].input_clamped;
    }

    const fuzzy::SugenoResult inference = fuzzy::evaluateSugeno3D(
        memberships[0], memberships[1], memberships[2],
        config_.target_velocity_singletons);
    if (!inference.valid || !std::isfinite(inference.value)) return output;

    // The configured singleton contract already limits the weighted average to
    // [-1,+1]. Clamp defensively at the public physical-unit boundary.
    output.target_velocity_normalized =
        std::clamp(inference.value, -1.0F, 1.0F);
    output.target_velocity_rad_s =
        output.target_velocity_normalized * config_.target_velocity_limit_rad_s;
    output.rule_weight_sum = inference.weight_sum;
    output.max_rule_weight = inference.max_rule_weight;
    output.valid = std::isfinite(output.target_velocity_rad_s) &&
                   std::isfinite(output.rule_weight_sum) &&
                   std::isfinite(output.max_rule_weight);
    if (!output.valid) return FuzzyBalanceOutput{};
    return output;
  }

 private:
  FuzzyBalanceConfig config_{};
};

}  // namespace triwhirl
