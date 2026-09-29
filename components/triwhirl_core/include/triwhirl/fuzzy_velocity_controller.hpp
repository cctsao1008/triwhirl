#pragma once

#include "triwhirl/fuzzy.hpp"

namespace triwhirl {

struct FuzzyVelocityControllerConfig {
  // Input universes. Values beyond these magnitudes are fuzzified at the
  // corresponding end shoulder rather than extrapolated.
  float error_scale_rad_s = 30.0F;
  float error_rate_scale_rad_s2 = 300.0F;
  float vq_limit_v = 4.0F;
};

struct FuzzyVelocityControllerOutput {
  bool valid = false;
  bool input_clamped = false;
  float error_rad_s = 0.0F;
  float error_rate_rad_s2 = 0.0F;
  float normalized_error = 0.0F;
  float normalized_error_rate = 0.0F;
  float normalized_vq = 0.0F;
  float vq_v = 0.0F;
  float max_rule_weight = 0.0F;
};

bool validFuzzyVelocityControllerConfig(
    const FuzzyVelocityControllerConfig& config);

// Stateless fuzzy inference surface. This is intentionally exposed so the
// rule surface can be exhaustively tested without controller history.
FuzzyVelocityControllerOutput evaluateFuzzyVelocityController(
    const FuzzyVelocityControllerConfig& config, float error_rad_s,
    float error_rate_rad_s2);

class FuzzyVelocityController {
 public:
  explicit FuzzyVelocityController(
      FuzzyVelocityControllerConfig config = {});

  void reset();
  const FuzzyVelocityControllerConfig& config() const { return config_; }

  FuzzyVelocityControllerOutput step(float target_rad_s,
                                     float measured_rad_s, float dt_s);

 private:
  FuzzyVelocityControllerConfig config_{};
  bool have_previous_error_ = false;
  float previous_error_rad_s_ = 0.0F;
};

}  // namespace triwhirl
