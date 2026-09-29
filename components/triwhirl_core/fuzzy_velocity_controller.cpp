#include "triwhirl/fuzzy_velocity_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace triwhirl {
namespace {

// Linguistic rows/columns: NL, NS, ZE, PS, PL.
// Rows are velocity error; columns are error rate. Error is deliberately the
// dominant term while error-rate anticipation softens or strengthens the
// command. The surface is exactly antisymmetric around the origin.
constexpr std::array<float, fuzzy::kFiveTermCount * fuzzy::kFiveTermCount>
    kVelocitySingletons{
        -1.00F, -1.00F, -1.00F, -0.75F, -0.50F,
        -1.00F, -0.75F, -0.50F, -0.25F,  0.00F,
        -0.50F, -0.25F,  0.00F,  0.25F,  0.50F,
         0.00F,  0.25F,  0.50F,  0.75F,  1.00F,
         0.50F,  0.75F,  1.00F,  1.00F,  1.00F,
    };

}  // namespace

bool validFuzzyVelocityControllerConfig(
    const FuzzyVelocityControllerConfig& config) {
  return std::isfinite(config.error_scale_rad_s) &&
         std::isfinite(config.error_rate_scale_rad_s2) &&
         std::isfinite(config.vq_limit_v) && config.error_scale_rad_s > 0.0F &&
         config.error_rate_scale_rad_s2 > 0.0F && config.vq_limit_v > 0.0F;
}

FuzzyVelocityControllerOutput evaluateFuzzyVelocityController(
    const FuzzyVelocityControllerConfig& config, const float error_rad_s,
    const float error_rate_rad_s2) {
  FuzzyVelocityControllerOutput output{};
  output.error_rad_s = error_rad_s;
  output.error_rate_rad_s2 = error_rate_rad_s2;

  if (!validFuzzyVelocityControllerConfig(config) ||
      !std::isfinite(error_rad_s) || !std::isfinite(error_rate_rad_s2)) {
    return output;
  }

  output.normalized_error = error_rad_s / config.error_scale_rad_s;
  output.normalized_error_rate =
      error_rate_rad_s2 / config.error_rate_scale_rad_s2;

  const fuzzy::FiveTermMembership error_membership =
      fuzzy::fiveTermMembership(output.normalized_error);
  const fuzzy::FiveTermMembership rate_membership =
      fuzzy::fiveTermMembership(output.normalized_error_rate);
  output.input_clamped =
      error_membership.input_clamped || rate_membership.input_clamped;

  const fuzzy::Sugeno2DResult inference = fuzzy::evaluateSugeno2D(
      error_membership, rate_membership, kVelocitySingletons);
  if (!inference.valid) {
    return output;
  }

  output.normalized_vq = std::clamp(inference.value, -1.0F, 1.0F);
  output.vq_v =
      std::clamp(output.normalized_vq * config.vq_limit_v,
                 -config.vq_limit_v, config.vq_limit_v);
  output.max_rule_weight = inference.max_rule_weight;
  output.valid = std::isfinite(output.vq_v);
  return output;
}

FuzzyVelocityController::FuzzyVelocityController(
    FuzzyVelocityControllerConfig config)
    : config_(config) {}

void FuzzyVelocityController::reset() {
  have_previous_error_ = false;
  previous_error_rad_s_ = 0.0F;
}

FuzzyVelocityControllerOutput FuzzyVelocityController::step(
    const float target_rad_s, const float measured_rad_s, const float dt_s) {
  if (!std::isfinite(target_rad_s) || !std::isfinite(measured_rad_s) ||
      !std::isfinite(dt_s) || !(dt_s > 0.0F)) {
    return FuzzyVelocityControllerOutput{};
  }

  const float error = target_rad_s - measured_rad_s;
  float error_rate = 0.0F;
  if (have_previous_error_) {
    error_rate = (error - previous_error_rad_s_) / dt_s;
  }

  previous_error_rad_s_ = error;
  have_previous_error_ = true;
  return evaluateFuzzyVelocityController(config_, error, error_rate);
}

}  // namespace triwhirl
