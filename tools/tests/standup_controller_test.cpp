#include <cassert>
#include <cmath>

#include "triwhirl/standup_controller.hpp"
#include "triwhirl/upright_geometry.hpp"

namespace {

float degToRad(float deg) { return deg * triwhirl::kPi / 180.0F; }

void testSwingAndCaptureLaw() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  triwhirl::StandupController controller(config);

  triwhirl::StandupControllerInput input{};
  input.valid = true;
  input.now_us = 1000U;
  input.theta_rad = degToRad(0.0F);
  input.theta_rate_rad_s = 0.5F;
  input.wheel_rate_rad_s = 0.0F;
  controller.reset(input);

  auto output = controller.update(input);
  assert(output.valid);
  assert(output.phase == triwhirl::StandupPhase::kSwingHigh);
  assert(std::fabs(output.vq_v - 0.42F) < 1.0e-6F);

  input.now_us += 1000U;
  input.theta_rad = degToRad(68.0F - 12.0F);
  input.theta_rate_rad_s = -0.4F;
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kSwingLow);
  assert(std::fabs(output.vq_v + 0.168F) < 1.0e-6F);

  input.now_us += 1000U;
  input.theta_rad = degToRad(68.0F - 8.0F);
  input.theta_rate_rad_s = 0.0F;
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(output.valid);
  assert(std::isfinite(output.target_velocity_rad_s));
  assert(std::isfinite(output.vq_v));
  assert(std::fabs(output.vq_v) <= config.vq_limit_v + 1.0e-6F);
}

void testPeriodicVerticesShareBalanceLaw() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  triwhirl::StandupController controller(config);

  triwhirl::StandupControllerInput input{};
  input.valid = true;
  input.now_us = 1000U;
  input.theta_rate_rad_s = 0.0F;
  input.wheel_rate_rad_s = 0.0F;

  input.theta_rad = degToRad(68.0F + 3.0F);
  controller.reset(input);
  auto a = controller.update(input);
  assert(a.phase == triwhirl::StandupPhase::kBalance);

  input.now_us += 1000U;
  input.theta_rad = degToRad(-52.0F + 3.0F);
  controller.reset(input);
  auto b = controller.update(input);
  assert(b.phase == triwhirl::StandupPhase::kBalance);
  assert(std::fabs(a.theta_error_rad - b.theta_error_rad) < 1.0e-5F);
}

void testStableTransitionAndRecovery() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  triwhirl::StandupController controller(config);

  triwhirl::StandupControllerInput input{};
  input.valid = true;
  input.now_us = 1000U;
  input.theta_rad = degToRad(69.0F);
  input.theta_rate_rad_s = 0.0F;
  input.wheel_rate_rad_s = 0.0F;
  controller.reset(input);
  auto output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(!output.stable);

  input.now_us += config.stable_delay_us + 1000U;
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(output.stable);

  input.now_us += 1000U;
  input.theta_rad = degToRad(68.0F + 25.0F);
  input.theta_rate_rad_s = 0.3F;
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kSwingHigh);
  assert(!output.stable);
}

}  // namespace

int main() {
  testSwingAndCaptureLaw();
  testPeriodicVerticesShareBalanceLaw();
  testStableTransitionAndRecovery();
  return 0;
}
