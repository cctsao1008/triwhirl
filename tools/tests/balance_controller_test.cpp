#include <cassert>
#include <cmath>
#include <initializer_list>

#include "triwhirl/balance_controller.hpp"
#include "triwhirl/upright_geometry.hpp"

namespace {

constexpr float deg(const float value) {
  return value * triwhirl::kPi / 180.0F;
}

}  // namespace

int main() {
  triwhirl::BalanceControllerConfig config{};
  config.k_theta = 5.0F;
  config.k_rate = 0.75F;
  config.k_wheel = -0.05F;
  config.theta_reference_rad = deg(68.0F);
  config.capture_angle_rad = deg(6.0F);
  config.fall_angle_rad = deg(24.0F);
  config.vq_limit_v = 1.2F;
  config.wheel_rate_limit_rad_s = 40.0F;
  assert(triwhirl::validBalanceControllerConfig(config));

  triwhirl::BalanceControllerInput input{};
  input.theta_rad = deg(70.0F);
  input.theta_rate_rad_s = 0.4F;
  input.wheel_rate_rad_s = -3.0F;
  const auto output = triwhirl::evaluateBalanceController(config, input);
  assert(output.valid);
  assert(output.capture_ready);
  assert(output.inside_envelope);
  assert(std::fabs(output.theta_error_rad - deg(2.0F)) < 1.0e-5F);
  const float expected =
      -(config.k_theta * deg(2.0F) + config.k_rate * 0.4F +
        config.k_wheel * -3.0F);
  assert(std::fabs(output.vq_unsaturated_v - expected) < 1.0e-5F);
  assert(std::fabs(output.vq_v - expected) < 1.0e-5F);

  // The same local state must produce the same command at all three physical
  // upright vertices separated by 120 degrees.
  for (const float offset_deg : {0.0F, 120.0F, -120.0F}) {
    input.theta_rad = deg(70.0F + offset_deg);
    const auto periodic = triwhirl::evaluateBalanceController(config, input);
    assert(periodic.valid);
    assert(std::fabs(periodic.theta_error_rad - deg(2.0F)) < 1.0e-5F);
    assert(std::fabs(periodic.vq_v - expected) < 1.0e-5F);
  }

  input.theta_rad = deg(90.0F);
  const auto outside_capture =
      triwhirl::evaluateBalanceController(config, input);
  assert(outside_capture.valid);
  assert(!outside_capture.capture_ready);
  assert(outside_capture.inside_envelope);

  input.theta_rad = deg(100.0F);
  const auto fallen = triwhirl::evaluateBalanceController(config, input);
  assert(fallen.valid);
  assert(!fallen.inside_envelope);

  input.theta_rad = deg(68.0F);
  input.theta_rate_rad_s = 100.0F;
  input.wheel_rate_rad_s = 0.0F;
  const auto saturated = triwhirl::evaluateBalanceController(config, input);
  assert(saturated.valid);
  assert(std::fabs(saturated.vq_v + config.vq_limit_v) < 1.0e-6F);

  input.theta_rate_rad_s = 0.0F;
  input.wheel_rate_rad_s = 41.0F;
  const auto overspeed = triwhirl::evaluateBalanceController(config, input);
  assert(overspeed.valid);
  assert(!overspeed.capture_ready);
  assert(!overspeed.inside_envelope);

  auto invalid = config;
  invalid.capture_angle_rad = invalid.fall_angle_rad;
  assert(!triwhirl::validBalanceControllerConfig(invalid));

  invalid = config;
  invalid.k_theta = 0.0F;
  invalid.k_rate = 0.0F;
  invalid.k_wheel = 0.0F;
  assert(!triwhirl::validBalanceControllerConfig(invalid));

  return 0;
}
