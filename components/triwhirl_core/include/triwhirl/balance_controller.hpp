#pragma once

namespace triwhirl {

// One state-feedback law is shared by all three physical upright vertices.
// theta_reference_rad selects one representative equilibrium; the 120-degree
// periodic coordinate maps the other two vertices into the same local error.
struct BalanceControllerConfig {
  float k_theta = 0.0F;
  float k_rate = 0.0F;
  float k_wheel = 0.0F;
  float theta_reference_rad = 0.0F;
  float capture_angle_rad = 0.0F;
  float fall_angle_rad = 0.0F;
  float vq_limit_v = 0.0F;
  float wheel_rate_limit_rad_s = 0.0F;
};

struct BalanceControllerInput {
  float theta_rad = 0.0F;
  float theta_rate_rad_s = 0.0F;
  float wheel_rate_rad_s = 0.0F;
};

struct BalanceControllerOutput {
  bool valid = false;
  bool capture_ready = false;
  bool inside_envelope = false;
  float theta_error_rad = 0.0F;
  float vq_unsaturated_v = 0.0F;
  float vq_v = 0.0F;
};

bool validBalanceControllerConfig(const BalanceControllerConfig& config);
BalanceControllerOutput evaluateBalanceController(
    const BalanceControllerConfig& config,
    const BalanceControllerInput& input);

}  // namespace triwhirl
