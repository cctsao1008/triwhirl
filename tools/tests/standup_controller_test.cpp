#include <cassert>
#include <cmath>

#include "triwhirl/standup_controller.hpp"
#include "triwhirl/upright_geometry.hpp"

namespace {

float degToRad(const float deg) { return deg * triwhirl::kPi / 180.0F; }

triwhirl::StandupControllerInput makeInput(const std::uint32_t now_us,
                                           const float theta_deg,
                                           const float rate_rad_s,
                                           const float wheel_rad_s = 0.0F) {
  triwhirl::StandupControllerInput input{};
  input.valid = true;
  input.now_us = now_us;
  input.theta_rad = degToRad(theta_deg);
  input.theta_rate_rad_s = rate_rad_s;
  input.wheel_rate_rad_s = wheel_rad_s;
  return input;
}

void testTunedDefaults() {
  const triwhirl::StandupControllerConfig config{};
  assert(std::fabs(config.lqr_k_angle_unstable + 8.0F) < 1.0e-6F);
  assert(std::fabs(config.lqr_k_rate_unstable - 0.35F) < 1.0e-6F);
  assert(std::fabs(config.lqr_k_wheel_unstable - 0.30F) < 1.0e-6F);
  assert(std::fabs(config.lqr_k_wheel_settling - 1.60F) < 1.0e-6F);
  assert(std::fabs(config.velocity_p_unstable - 0.035F) < 1.0e-6F);
  assert(std::fabs(config.velocity_i_unstable - 0.150F) < 1.0e-6F);
  assert(std::fabs(config.stable_angle_rad - degToRad(5.0F)) < 1.0e-6F);
  assert(config.stable_delay_us == 1000000U);
}

void testSwingAndPreSettlingReleaseHysteresis() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  triwhirl::StandupController controller(config);

  auto input = makeInput(1000U, 0.0F, 0.5F);
  controller.reset(input);
  auto output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kSwingHigh);
  assert(std::fabs(output.vq_v - 0.42F) < 1.0e-6F);

  input = makeInput(2000U, 60.0F, 0.5F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);

  input = makeInput(3000U, 57.5F, 0.5F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);

  input = makeInput(4000U, 55.5F, 0.5F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kSwingLow);
}

void testWideReversalDoesNotLatchSettling() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  triwhirl::StandupController controller(config);

  auto input = makeInput(1000U, 60.0F, -1.0F);
  controller.reset(input);
  auto output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);

  input = makeInput(2000U, 57.5F, -1.0F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);

  input = makeInput(3000U, 55.5F, -1.0F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kSwingLow);
}

void testVendorGyroEnvelopeOnApproach() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  config.lqr_k_angle_unstable = 0.0F;
  config.lqr_k_rate_unstable = 0.35F;
  config.lqr_k_wheel_unstable = 0.0F;
  config.velocity_target_limit_rad_s = 100.0F;
  triwhirl::StandupController controller(config);

  auto input = makeInput(1000U, 66.0F, 12.0F);
  controller.reset(input);
  auto output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(std::fabs(output.target_velocity_rad_s + 35.0F) < 0.03F);

  input.now_us += 1000U;
  output = controller.update(input);
  assert(std::fabs(output.target_velocity_rad_s + 56.0F) < 0.03F);
}

void testBilateralSettlingPersistsUntilTrueFall() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  config.lqr_k_angle_unstable = -8.0F;
  config.lqr_k_rate_unstable = 0.0F;
  config.lqr_k_wheel_unstable = 0.0F;
  config.lqr_k_wheel_settling = 0.0F;
  config.velocity_p_unstable = 0.0F;
  config.velocity_i_unstable = 0.0F;
  config.velocity_p_recovery_unstable = 0.0F;
  config.velocity_i_recovery_unstable = 0.0F;
  config.recovery_rate_damping_v_per_rad_s = 0.0F;
  config.velocity_target_limit_rad_s = 100.0F;
  triwhirl::StandupController controller(config);

  auto input = makeInput(1000U, 70.0F, -1.0F);
  controller.reset(input);
  auto output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(output.target_velocity_rad_s < 0.0F);

  input = makeInput(2000U, 67.8F, -1.0F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(output.target_velocity_rad_s > 1.5F);

  input = makeInput(3000U, 68.5F, 1.0F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(output.target_velocity_rad_s < -3.9F);

  input = makeInput(4000U, 67.5F, -1.0F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(output.target_velocity_rad_s > 3.9F);

  input = makeInput(5000U, 98.0F, 0.0F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);

  input = makeInput(6000U, 122.0F, 0.0F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);

  input = makeInput(7000U, 124.0F, 0.5F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kSwingHigh);
}

void testSettlingClearsApproachIntegralBias() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  config.lqr_k_angle_unstable = -8.0F;
  config.lqr_k_rate_unstable = 0.0F;
  config.lqr_k_wheel_unstable = 0.0F;
  config.velocity_p_unstable = 0.0F;
  config.velocity_i_unstable = 20.0F;
  config.velocity_p_recovery_unstable = 0.0F;
  config.velocity_i_recovery_unstable = 0.0F;
  config.recovery_rate_damping_v_per_rad_s = 0.0F;
  triwhirl::StandupController controller(config);

  auto input = makeInput(1000U, 70.0F, -0.1F);
  controller.reset(input);
  auto output = controller.update(input);
  for (int i = 0; i < 20; ++i) {
    input.now_us += 1000U;
    output = controller.update(input);
  }
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(std::fabs(output.velocity_integral_v) > 0.1F);

  input.now_us += 1000U;
  input.theta_rad = degToRad(67.8F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(std::fabs(output.velocity_integral_v) < 1.0e-6F);
}

void testStableIsStatusOnlyAndReferenceStaysFixed() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  config.stable_delay_us = 10000U;
  config.lqr_k_angle_unstable = -8.0F;
  config.lqr_k_rate_unstable = 0.0F;
  config.lqr_k_wheel_unstable = 0.0F;
  config.lqr_k_wheel_settling = 0.0F;
  config.velocity_p_unstable = 0.0F;
  config.velocity_i_unstable = 0.0F;
  config.velocity_p_recovery_unstable = 0.0F;
  config.velocity_i_recovery_unstable = 0.0F;
  config.recovery_rate_damping_v_per_rad_s = 0.0F;
  triwhirl::StandupController controller(config);

  auto input = makeInput(1000U, 69.0F, 0.0F);
  controller.reset(input);
  auto output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(!output.stable);
  assert(std::fabs(output.theta_reference_rad - degToRad(68.0F)) < 1.0e-6F);

  input.now_us = 12000U;
  output = controller.update(input);
  assert(output.stable);
  assert(std::fabs(output.theta_reference_rad - degToRad(68.0F)) < 1.0e-6F);
  assert(std::fabs(output.theta_error_rad - degToRad(1.0F)) < 1.0e-5F);
  assert(output.target_velocity_rad_s < -7.9F);

  input = makeInput(13000U, 67.0F, 0.0F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(output.stable);
  assert(output.target_velocity_rad_s > 7.9F);
  assert(std::fabs(output.theta_reference_rad - degToRad(68.0F)) < 1.0e-6F);

  input = makeInput(14000U, 74.0F, 0.0F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(!output.stable);
  assert(output.target_velocity_rad_s < 0.0F);

  input = makeInput(15000U, 98.0F, 0.0F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
}

void testReactionWheelDampingPolarity() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  config.lqr_k_angle_unstable = 0.0F;
  config.lqr_k_rate_unstable = 0.0F;
  config.lqr_k_wheel_unstable = 0.0F;
  config.lqr_k_wheel_settling = 0.0F;
  config.velocity_p_unstable = 0.0F;
  config.velocity_i_unstable = 0.0F;
  config.velocity_p_recovery_unstable = 0.0F;
  config.velocity_i_recovery_unstable = 0.0F;
  config.recovery_rate_damping_v_per_rad_s = 2.0F;
  config.recovery_rate_damping_limit_v = 2.5F;
  config.vq_limit_v = 4.0F;
  triwhirl::StandupController controller(config);

  auto input = makeInput(1000U, 70.0F, -1.0F);
  controller.reset(input);
  auto output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);

  input = makeInput(2000U, 67.8F, -1.0F);
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(output.filtered_rate_rad_s < 0.0F);
  assert(output.vq_target_v > 0.5F);

  input = makeInput(3000U, 67.9F, 1.0F);
  output = controller.update(input);
  input = makeInput(4000U, 68.1F, 1.0F);
  output = controller.update(input);
  assert(output.filtered_rate_rad_s > 0.0F);
  assert(output.vq_target_v < -0.5F);
}

void testSettlingWheelFeedbackPreservesRestoringVq() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  config.lqr_k_angle_unstable = -8.0F;
  config.lqr_k_rate_unstable = 0.0F;
  config.lqr_k_wheel_unstable = 0.0F;
  config.lqr_k_wheel_settling = 1.60F;
  config.velocity_p_unstable = 0.0F;
  config.velocity_i_unstable = 0.0F;
  config.velocity_p_recovery_unstable = 0.035F;
  config.velocity_i_recovery_unstable = 0.0F;
  config.recovery_rate_damping_v_per_rad_s = 0.0F;
  config.velocity_target_limit_rad_s = 80.0F;
  config.vq_limit_v = 4.0F;
  triwhirl::StandupController controller(config);

  auto input = makeInput(1000U, 70.0F, -1.0F, -7.0F);
  controller.reset(input);
  auto output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);

  input = makeInput(2000U, 67.8F, -1.0F, -7.0F);
  output = controller.update(input);
  assert(output.settling);

  // Reproduce the failure geometry seen in standup-20260926-205750: a tiny
  // positive error with the wheel already carrying negative momentum. A
  // position-only target would yield a positive velocity error and wrong-sign
  // Vq here. +1.6*wheel keeps the commanded wheel acceleration restoring.
  input = makeInput(3000U, 68.16F, 0.0F, -6.95F);
  output = controller.update(input);
  assert(output.settling);
  assert(output.target_velocity_rad_s < input.wheel_rate_rad_s);
  assert(output.velocity_error_rad_s < 0.0F);
  assert(output.vq_target_v < 0.0F);

  input = makeInput(4000U, 67.84F, 0.0F, 6.95F);
  output = controller.update(input);
  assert(output.settling);
  assert(output.target_velocity_rad_s > input.wheel_rate_rad_s);
  assert(output.velocity_error_rad_s > 0.0F);
  assert(output.vq_target_v > 0.0F);
}

void testVelocityOutputRamp() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  triwhirl::StandupController controller(config);

  auto input = makeInput(1000U, 60.0F, 0.0F);
  controller.reset(input);
  input.now_us += 1000U;
  auto first = controller.update(input);
  assert(first.phase == triwhirl::StandupPhase::kBalance);
  assert(std::fabs(first.vq_v) <= 1.000001F);

  input.now_us += 1000U;
  auto second = controller.update(input);
  assert(second.phase == triwhirl::StandupPhase::kBalance);
  assert(std::fabs(second.vq_v - first.vq_v) <= 1.000001F);
}

void testConditionalAntiWindup() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  config.lqr_k_angle_unstable = -10.0F;
  config.lqr_k_rate_unstable = 0.0F;
  config.lqr_k_wheel_unstable = 0.0F;
  config.velocity_p_unstable = 0.10F;
  config.velocity_i_unstable = 100.0F;
  config.velocity_target_limit_rad_s = 60.0F;
  config.vq_limit_v = 1.0F;
  config.recovery_rate_damping_limit_v = 0.0F;
  triwhirl::StandupController controller(config);

  auto input = makeInput(1000U, 60.0F, 0.0F);
  controller.reset(input);
  auto output = controller.update(input);
  for (int i = 0; i < 100; ++i) {
    input.now_us += 1000U;
    output = controller.update(input);
  }
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(std::fabs(output.vq_target_v - config.vq_limit_v) < 1.0e-6F);
  assert(std::fabs(output.velocity_integral_v) < 1.0e-6F);
}

void testPeriodicVerticesShareBalanceLaw() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  triwhirl::StandupController controller(config);

  auto input = makeInput(1000U, 71.0F, 0.0F);
  controller.reset(input);
  const auto a = controller.update(input);
  assert(a.phase == triwhirl::StandupPhase::kBalance);

  input = makeInput(2000U, -49.0F, 0.0F);
  controller.reset(input);
  const auto b = controller.update(input);
  assert(b.phase == triwhirl::StandupPhase::kBalance);
  assert(std::fabs(a.theta_error_rad - b.theta_error_rad) < 1.0e-5F);
}

}  // namespace

int main() {
  testTunedDefaults();
  testSwingAndPreSettlingReleaseHysteresis();
  testWideReversalDoesNotLatchSettling();
  testVendorGyroEnvelopeOnApproach();
  testBilateralSettlingPersistsUntilTrueFall();
  testSettlingClearsApproachIntegralBias();
  testStableIsStatusOnlyAndReferenceStaysFixed();
  testReactionWheelDampingPolarity();
  testSettlingWheelFeedbackPreservesRestoringVq();
  testVelocityOutputRamp();
  testConditionalAntiWindup();
  testPeriodicVerticesShareBalanceLaw();
  return 0;
}