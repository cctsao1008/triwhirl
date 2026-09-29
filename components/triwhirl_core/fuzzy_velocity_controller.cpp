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
    kCorrectionSingletons{
        -1.00F, -1.00F, -1.00F, -0.75F, -0.50F,
        -1.00F, -0.75F, -0.50F, -0.25F,  0.00F,
        -0.50F, -0.25F,  0.00F,  0.25F,  0.50F,
         0.00F,  0.25F,  0.50F,  0.75F,  1.00F,
         0.50F,  0.75F,  1.00F,  1.00F,  1.00F,
    };

std::array<float, fuzzy::kFiveTermCount> holdSingletons(
    const float hold_vq_at_scale_v) {
  std::array<float, fuzzy::kFiveTermCount> result{};
  for (std::size_t i = 0; i < result.size(); ++i) {
    result[i] = fuzzy::kFiveTermCenters[i] * hold_vq_at_scale_v;
  }
  return result;
}

}  // namespace

bool validFuzzyVelocityControllerConfig(
    const FuzzyVelocityControllerConfig& config) {
  const bool finite = std::isfinite(config.error_scale_rad_s) &&
                      std::isfinite(config.error_rate_scale_rad_s2) &&
                      std::isfinite(config.wheel_rate_scale_rad_s) &&
                      std::isfinite(config.hold_vq_at_scale_v) &&
                      std::isfinite(config.correction_vq_limit_v) &&
                      std::isfinite(config.vq_limit_v);
  return finite && config.error_scale_rad_s > 0.0F &&
         config.error_rate_scale_rad_s2 > 0.0F &&
         config.wheel_rate_scale_rad_s > 0.0F &&
         config.hold_vq_at_scale_v >= 0.0F &&
         config.correction_vq_limit_v > 0.0F && config.vq_limit_v > 0.0F &&
         config.hold_vq_at_scale_v <= config.vq_limit_v &&
         config.correction_vq_limit_v <= config.vq_limit_v;
}

FuzzyVelocityControllerOutput evaluateFuzzyVelocityController(
    const FuzzyVelocityControllerConfig& config, const float error_rad_s,
    const float error_rate_rad_s2, const float wheel_rate_rad_s) {
  FuzzyVelocityControllerOutput output{};
  output.error_rad_s = error_rad_s;
  output.error_rate_rad_s2 = error_rate_rad_s2;
  output.wheel_rate_rad_s = wheel_rate_rad_s;

  if (!validFuzzyVelocityControllerConfig(config) ||
      !std::isfinite(error_rad_s) || !std::isfinite(error_rate_rad_s2) ||
      !std::isfinite(wheel_rate_rad_s)) {
    return output;
  }

  output.normalized_error = error_rad_s / config.error_scale_rad_s;
  output.normalized_error_rate =
      error_rate_rad_s2 / config.error_rate_scale_rad_s2;
  output.normalized_wheel_rate =
      wheel_rate_rad_s / config.wheel_rate_scale_rad_s;

  const fuzzy::FiveTermMembership error_membership =
      fuzzy::fiveTermMembership(output.normalized_error);
  const fuzzy::FiveTermMembership rate_membership =
      fuzzy::fiveTermMembership(output.normalized_error_rate);
  const fuzzy::FiveTermMembership wheel_membership =
      fuzzy::fiveTermMembership(output.normalized_wheel_rate);
  output.input_clamped = error_membership.input_clamped ||
                         rate_membership.input_clamped ||
                         wheel_membership.input_clamped;

  const fuzzy::SugenoResult hold = fuzzy::evaluateSugeno1D(
      wheel_membership, holdSingletons(config.hold_vq_at_scale_v));
  const fuzzy::SugenoResult correction = fuzzy::evaluateSugeno2D(
      error_membership, rate_membership, kCorrectionSingletons);
  if (!hold.valid || !correction.valid) return output;

  output.hold_vq_v = hold.value;
  output.correction_vq_v =
      std::clamp(correction.value, -1.0F, 1.0F) *
      config.correction_vq_limit_v;
  output.vq_unclamped_v = output.hold_vq_v + output.correction_vq_v;
  output.vq_v = std::clamp(output.vq_unclamped_v, -config.vq_limit_v,
                           config.vq_limit_v);
  output.vq_saturated = output.vq_v != output.vq_unclamped_v;
  output.max_rule_weight =
      std::max(hold.max_rule_weight, correction.max_rule_weight);
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
  return evaluateFuzzyVelocityController(config_, error, error_rate,
                                         measured_rad_s);
}

}  // namespace triwhirl
