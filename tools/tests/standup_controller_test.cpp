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

  // Once captured, a small excursion beyond the 9-deg entry threshold must not
  // immediately throw the controller back into swing-up. This is the exact
  // handoff case observed on the physical unit (about 9.4/10.6 deg).
  input.now_us += 1000U;
  input.theta_rad = degToRad(68.0F - 10.5F);
  input.theta_rate_rad_s = 0.1F;
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);

  input.now_us += 1000U;
  input.theta_rad = degToRad(68.0F - 12.5F);
  input.theta_rate_rad_s = -0.2F;
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kSwingLow);
}

void testVelocityOutputRamp() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  assert(std::fabs(config.velocity_output_ramp_v_s - 1000.0F) < 1.0e-6F);
  triwhirl::StandupController controller(config);

  triwhirl::StandupControllerInput input{};
  input.valid = true;
  input.now_us = 1000U;
  input.theta_rad = degToRad(60.0F);  // -8 deg: enter Balance with a large demand.
  input.theta_rate_rad_s = 0.0F;
  input.wheel_rate_rad_s = 0.0F;
  controller.reset(input);

  input.now_us += 1000U;
  auto first = controller.update(input);
  assert(first.phase == triwhirl::StandupPhase::kBalance);
  assert(first.valid);
  // 1000 V/s at 1 ms permits at most a 1 V step from the reset output.
  assert(std::fabs(first.vq_v) <= 1.000001F);

  input.now_us += 1000U;
  auto second = controller.update(input);
  assert(second.phase == triwhirl::StandupPhase::kBalance);
  assert(second.valid);
  assert(std::fabs(second.vq_v - first.vq_v) <= 1.000001F);
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
  testVelocityOutputRamp();
  testPeriodicVerticesShareBalanceLaw();
  testStableTransitionAndRecovery();
  return 0;
}
