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

void testVendorGyroEnvelopeOnCapture() {
  triwhirl::StandupControllerConfig config{};
  config.theta_reference_rad = degToRad(68.0F);
  assert(std::fabs(config.gyro_rate_limit_rad_s - degToRad(250.0F)) < 1.0e-5F);
  triwhirl::StandupController controller(config);

  triwhirl::StandupControllerInput input{};
  input.valid = true;
  input.now_us = 1000U;
  input.theta_rad = degToRad(45.0F);
  input.theta_rate_rad_s = 12.0F;  // wider runtime IMU sees > vendor +/-250 dps
  input.wheel_rate_rad_s = 0.0F;
  controller.reset(input);
  auto swing = controller.update(input);
  assert(swing.phase == triwhirl::StandupPhase::kSwingHigh);

  // At the first Balance tick the golden firmware has just executed Gyro=0 in
  // the swing branch. Therefore its first filtered rate is
  // 0.4 * clamp(+12 rad/s, +250 deg/s) = +100 deg/s.
  input.now_us += 1000U;
  input.theta_rad = degToRad(68.0F);
  auto capture = controller.update(input);
  assert(capture.phase == triwhirl::StandupPhase::kBalance);
  assert(capture.valid);
  const float expected_target = -0.92F * 100.0F;
  assert(std::fabs(capture.target_velocity_rad_s - expected_target) < 0.02F);
  assert(std::fabs(capture.target_velocity_rad_s) < config.velocity_target_limit_rad_s);

  // A second saturated sample follows the exact seller 0.6/0.4 recurrence:
  // Gyro = 0.6*100 + 0.4*250 = 160 deg/s.
  input.now_us += 1000U;
  auto second = controller.update(input);
  const float expected_second_target = -0.92F * 160.0F;
  // The target itself is limited to +/-140 rad/s, matching the golden firmware.
  assert(std::fabs(second.target_velocity_rad_s + 140.0F) < 1.0e-5F);
  assert(expected_second_target < -140.0F);
}

void testVendorStableCaptureAfterSwing() {
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

  // The golden outer loop spends this entire interval in torque-mode swing-up;
  // controllerLQR() is not called, so last_unstable_time is not refreshed.
  input.now_us += config.stable_delay_us + 1000U;
  auto swing = controller.update(input);
  assert(swing.phase == triwhirl::StandupPhase::kSwingHigh);

  // Arriving directly inside +/-5 deg after that swing immediately satisfies the
  // seller's stable-time test. It recenters target_angle and uses the stable
  // LQR/velocity-PI gain set on this very first capture call.
  input.now_us += 1000U;
  input.theta_rad = degToRad(67.0F);  // -1 deg relative to the nominal 68 deg.
  input.theta_rate_rad_s = 0.0F;
  input.wheel_rate_rad_s = 0.0F;
  const auto capture = controller.update(input);
  assert(capture.phase == triwhirl::StandupPhase::kBalance);
  assert(capture.stable);
  assert(std::fabs(capture.theta_reference_rad - degToRad(67.0F)) < 1.0e-5F);
  // controllerLQR() still uses the current call's original p_angle (-1 deg), so
  // stable K_angle=-6.5 produces +6.5 rad/s before the velocity PI.
  assert(std::fabs(capture.target_velocity_rad_s - 6.5F) < 0.02F);

  // The shifted reference takes effect on the next outer-loop iteration.
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
  testVendorGyroEnvelopeOnCapture();
  testVendorStableCaptureAfterSwing();
  testVelocityOutputRamp();
  testPeriodicVerticesShareBalanceLaw();
  testStableTransitionAndRecovery();
  return 0;
}
