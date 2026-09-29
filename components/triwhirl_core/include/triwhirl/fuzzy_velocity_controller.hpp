#pragma once

#include "triwhirl/fuzzy.hpp"

namespace triwhirl {

struct FuzzyVelocityControllerConfig {
  // Fuzzy input universes. Values outside the universe use end shoulders.
  float error_scale_rad_s = 30.0F;
  float error_rate_scale_rad_s2 = 300.0F;
  float wheel_rate_scale_rad_s = 60.0F;

  // Pure-fuzzy steady-speed hold map. At +/-wheel_rate_scale_rad_s the 1-D
  // Sugeno hold engine emits +/-hold_vq_at_scale_v. Zero disables hold bias.
  // This value must be calibrated from motor/wheel evidence before hardware use.
  float hold_vq_at_scale_v = 0.0F;

  // The 2-D error/error-rate engine contributes a bounded correction around
  // the fuzzy hold map. The final Vq hard clamp remains deterministic safety.
  float correction_vq_limit_v = 2.0F;
  float vq_limit_v = 4.0F;
};

struct FuzzyVelocityControllerOutput {
  bool valid = false;
  bool input_clamped = false;
  bool vq_saturated = false;
  float error_rad_s = 0.0F;
  float error_rate_rad_s2 = 0.0F;
  float wheel_rate_rad_s = 0.0F;
  float normalized_error = 0.0F;
  float normalized_error_rate = 0.0F;
  float normalized_wheel_rate = 0.0F;
  float hold_vq_v = 0.0F;
  float correction_vq_v = 0.0F;
  float vq_unclamped_v = 0.0F;
  float vq_v = 0.0F;
  float max_rule_weight = 0.0F;
};

bool validFuzzyVelocityControllerConfig(
    const FuzzyVelocityControllerConfig& config);

// Stateless composite pure-fuzzy inference. A 1-D wheel-rate engine supplies
// the steady-speed hold command and a 2-D error/error-rate engine supplies the
// corrective command. No PI/PID state or gain equation is used.
FuzzyVelocityControllerOutput evaluateFuzzyVelocityController(
    const FuzzyVelocityControllerConfig& config, float error_rad_s,
    float error_rate_rad_s2, float wheel_rate_rad_s);

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
