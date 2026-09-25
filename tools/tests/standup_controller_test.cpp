#include <cassert>
#include <cmath>

#include "triwhirl/standup_controller.hpp"
#include "triwhirl/upright_geometry.hpp"

namespace {

float degToRad(float deg) { return deg * triwhirl::kPi / 180.0F; }

void testTunedDefaults() {
  const triwhirl::StandupControllerConfig config{};
  assert(std::fabs(config.lqr_k_angle_unstable + 8.0F) < 1.0e-6F);
  assert(std::fabs(config.lqr_k_rate_unstable - 0.35F) < 1.0e-6F);
  assert(std::fabs(config.lqr_k_rate_recovery_unstable - 0.55F) < 1.0e-6F);
  assert(std::fabs(config.lqr_k_wheel_unstable - 0.30F) < 1.0e-6F);
  assert(std::fabs(config.lqr_k_angle_stable + 2.5F) < 1.0e-6F);
  assert(std::fabs(config.lqr_k_rate_stable - 0.35F) < 1.0e-6F);
  assert(std::fabs(config.lqr_k_wheel_stable - 0.20F) < 1.0e-6F);
  assert(std::fabs(config.velocity_p_unstable - 0.035F) < 1.0e-6F);
  assert(std::fabs(config.velocity_i_unstable - 0.150F) < 1.0e-6F);
  assert(std::fabs(config.velocity_p_stable - 0.018F) < 1.0e-6F);
  assert(std::fabs(config.velocity_i_stable - 0.100F) < 1.0e-6F);
  assert(std::fabs(config.velocity_target_limit_rad_s - 60.0F) < 1.0e-6F);
  assert(std::fabs(config.vq_limit_v - 3.0F) < 1.0e-6F);
}

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

void testVendorGyroEnvelopeOnCapture() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  assert(std::fabs(config.gyro_rate_limit_rad_s - degToRad(250.0F)) < 1.0e-5F);
  triwhirl::StandupController controller(config);

  triwhirl::StandupControllerInput input{};
  input.valid = true;
  input.now_us = 1000U;
  input.theta_rad = degToRad(45.0F);
  input.theta_rate_rad_s = 12.0F;
  input.wheel_rate_rad_s = 0.0F;
  controller.reset(input);
  auto swing = controller.update(input);
  assert(swing.phase == triwhirl::StandupPhase::kSwingHigh);

  input.now_us += 1000U;
  input.theta_rad = degToRad(68.0F);
  auto capture = controller.update(input);
  assert(capture.phase == triwhirl::StandupPhase::kBalance);
  assert(capture.valid);
  // At zero angle error with finite rate, recovery damping is selected.
  const float expected_target = -0.55F * 100.0F;
  assert(std::fabs(capture.target_velocity_rad_s - expected_target) < 0.02F);

  input.now_us += 1000U;
  auto second = controller.update(input);
  // 0.6*100 + 0.4*250 = 160 deg/s; -0.55*160 clips to the -60 limit.
  assert(std::fabs(second.target_velocity_rad_s + 60.0F) < 1.0e-5F);
}

void testStableCaptureAfterSwing() {
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

  input.now_us += config.stable_delay_us + 1000U;
  auto swing = controller.update(input);
  assert(swing.phase == triwhirl::StandupPhase::kSwingHigh);

  input.now_us += 1000U;
  input.theta_rad = degToRad(67.0F);
  input.theta_rate_rad_s = 0.0F;
  input.wheel_rate_rad_s = 0.0F;
  const auto capture = controller.update(input);
  assert(capture.phase == triwhirl::StandupPhase::kBalance);
  assert(capture.stable);
  assert(std::fabs(capture.theta_reference_rad - degToRad(67.0F)) < 1.0e-5F);
  assert(std::fabs(capture.target_velocity_rad_s - 2.5F) < 0.02F);

  input.now_us += 1000U;
  const auto next = controller.update(input);
  assert(next.phase == triwhirl::StandupPhase::kBalance);
  assert(next.stable);
  assert(std::fabs(next.theta_error_rad) < 1.0e-5F);
}

void testVelocityOutputRamp() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  assert(std::fabs(config.velocity_output_ramp_v_s - 1000.0F) < 1.0e-6F);
  triwhirl::StandupController controller(config);

  triwhirl::StandupControllerInput input{};
  input.valid = true;
  input.now_us = 1000U;
  input.theta_rad = degToRad(60.0F);
  input.theta_rate_rad_s = 0.0F;
  input.wheel_rate_rad_s = 0.0F;
  controller.reset(input);

  input.now_us += 1000U;
  auto first = controller.update(input);
  assert(first.phase == triwhirl::StandupPhase::kBalance);
  assert(first.valid);
  assert(std::fabs(first.vq_v) <= 1.000001F);

  input.now_us += 1000U;
  auto second = controller.update(input);
  assert(second.phase == triwhirl::StandupPhase::kBalance);
  assert(second.valid);
  assert(std::fabs(second.vq_v - first.vq_v) <= 1.000001F);
}

void testVelocityLoopResetsOnRecapture() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  // This test isolates recapture reset behavior. Keep proportional action out
  // of the output rail so anti-windup does not intentionally hold the integral.
  config.velocity_p_unstable = 0.0F;
  config.velocity_i_unstable = 4.0F;
  triwhirl::StandupController controller(config);

  triwhirl::StandupControllerInput input{};
  input.valid = true;
  input.now_us = 1000U;
  input.theta_rad = degToRad(60.0F);
  input.theta_rate_rad_s = 0.0F;
  input.wheel_rate_rad_s = -20.0F;
  controller.reset(input);

  auto output = controller.update(input);
  for (int i = 0; i < 100; ++i) {
    input.now_us += 1000U;
    output = controller.update(input);
  }
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(std::fabs(output.velocity_integral_v) > 0.1F);

  input.now_us += 1000U;
  input.theta_rad = degToRad(68.0F - 20.0F);
  input.theta_rate_rad_s = 0.5F;
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kSwingHigh);

  input.now_us += 1000U;
  input.theta_rad = degToRad(68.0F - 8.0F);
  input.theta_rate_rad_s = 0.0F;
  input.wheel_rate_rad_s = 0.0F;
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  assert(std::fabs(output.velocity_integral_v) < 0.1F);
}

void testConditionalAntiWindup() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  config.lqr_k_angle_unstable = -10.0F;
  config.lqr_k_rate_unstable = 0.0F;
  config.lqr_k_rate_recovery_unstable = 0.0F;
  config.lqr_k_wheel_unstable = 0.0F;
  // Proportional action alone is deliberately beyond the 1 V output rail. The
  // integral must therefore be held instead of accumulating in the same sign.
  config.velocity_p_unstable = 0.10F;
  config.velocity_i_unstable = 100.0F;
  config.velocity_target_limit_rad_s = 60.0F;
  config.vq_limit_v = 1.0F;
  triwhirl::StandupController controller(config);

  triwhirl::StandupControllerInput input{};
  input.valid = true;
  input.now_us = 1000U;
  input.theta_rad = degToRad(60.0F);
  input.theta_rate_rad_s = 0.0F;
  input.wheel_rate_rad_s = 0.0F;
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

void testRecoveryDampingLatchesAfterFirstCrossing() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  config.lqr_k_angle_unstable = 0.0F;
  config.lqr_k_rate_unstable = 0.35F;
  config.lqr_k_rate_recovery_unstable = 0.55F;
  config.lqr_k_wheel_unstable = 0.0F;
  config.velocity_p_unstable = 0.0F;
  config.velocity_i_unstable = 0.0F;
  config.velocity_target_limit_rad_s = 100.0F;
  triwhirl::StandupController controller(config);

  triwhirl::StandupControllerInput input{};
  input.valid = true;
  input.now_us = 1000U;
  input.theta_rad = degToRad(70.0F);  // +2 deg
  input.theta_rate_rad_s = -1.0F;
  input.wheel_rate_rad_s = 0.0F;
  controller.reset(input);

  auto output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);
  // First approach still uses 0.35 damping: filtered rate = -0.4 rad/s.
  const float first_expected = 0.35F * 0.4F * 180.0F / triwhirl::kPi;
  assert(std::fabs(output.target_velocity_rad_s - first_expected) < 0.02F);

  input.now_us += 1000U;
  input.theta_rad = degToRad(67.8F);  // Cross to -0.2 deg.
  input.theta_rate_rad_s = -1.0F;
  output = controller.update(input);
  assert(output.phase == triwhirl::StandupPhase::kBalance);

  // Return toward zero after the first crossing. The stronger 0.55 damping must
  // remain latched instead of dropping back to the 0.35 approach gain.
  input.now_us += 1000U;
  input.theta_rad = degToRad(67.0F);
  input.theta_rate_rad_s = 1.0F;
  output = controller.update(input);
  input.now_us += 1000U;
  input.theta_rad = degToRad(67.5F);
  input.theta_rate_rad_s = 1.0F;
  output = controller.update(input);
  assert(output.target_velocity_rad_s < -12.0F);
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
  testTunedDefaults();
  testSwingAndCaptureLaw();
  testVendorGyroEnvelopeOnCapture();
  testStableCaptureAfterSwing();
  testVelocityOutputRamp();
  testVelocityLoopResetsOnRecapture();
  testConditionalAntiWindup();
  testRecoveryDampingLatchesAfterFirstCrossing();
  testPeriodicVerticesShareBalanceLaw();
  testStableTransitionAndRecovery();
  return 0;
}