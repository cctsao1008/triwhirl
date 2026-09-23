#pragma once

#include <cstdint>

namespace triwhirl {

enum class StandupPhase : std::uint8_t {
  kIdle = 0,
  kSwingHigh,
  kSwingLow,
  kBalance,
};

struct StandupControllerConfig {
  // Local upright reference. The same 120-degree periodic coordinate covers all
  // three physical vertices.
  float theta_reference_rad = 0.0F;

  // TRC-V1.1 proven swing/balance handoff envelope. Enter Balance at 9 deg as
  // the seller firmware does, but once captured keep the local controller until
  // 12 deg. The small hysteresis prevents a marginal 9.x/10.x-deg excursion
  // from immediately replacing corrective balance torque with the swing pump.
  float balance_capture_rad = 0.1570796327F;  // 9 deg
  float balance_release_rad = 0.2094395102F;  // 12 deg
  float swing_near_rad = 0.3141592654F;       // 18 deg
  float pump_v_low = 0.168F;
  float pump_v_high = 0.42F;
  float rate_switch_rad_s = 0.03F;

  // Vendor LQR produces a reaction-wheel velocity target. The units are kept
  // compatible with the seller firmware: angle/rate in degrees(/s), wheel
  // velocity and target velocity in rad/s.
  float lqr_k_angle_unstable = -8.0F;
  float lqr_k_rate_unstable = 0.92F;
  float lqr_k_wheel_unstable = 1.6F;
  float lqr_k_angle_stable = -6.5F;
  float lqr_k_rate_stable = 0.9F;
  float lqr_k_wheel_stable = 1.5F;

  // Golden TRC-V1.1 configures the MPU6050 gyro for +/-250 deg/s. Our runtime
  // deliberately uses a wider sensor range, so clamp only the vendor standup
  // controller input to the same physical envelope before applying its 0.6/0.4
  // Gyro filter. This preserves the seller control law without reducing the
  // global IMU range used by diagnostics/identification.
  float gyro_rate_limit_rad_s = 4.36332313F;  // 250 deg/s

  float velocity_p_unstable = 0.035F;
  float velocity_i_unstable = 0.8F;
  float velocity_p_stable = 0.03F;
  float velocity_i_stable = 0.7F;
  float velocity_target_limit_rad_s = 140.0F;
  float vq_limit_v = 4.0F;
  // The supplied Arduino-FOC 2.1.1 library defaults
  // DEF_PID_VEL_RAMP=1000 V/s. Preserve that velocity-PI actuator slew limit.
  float velocity_output_ramp_v_s = 1000.0F;

  // In the seller controller, last_unstable_time is refreshed only while the
  // Balance branch is executing with |p_angle| > 5 deg. Swing-up does not
  // refresh it. Therefore a capture that arrives directly within 5 deg after a
  // sufficiently long swing can enter the stable gain set immediately and
  // recenter target_angle on that capture.
  float stable_angle_rad = 0.0872664626F;  // 5 deg
  std::uint32_t stable_delay_us = 1000000U;
  std::uint32_t momentum_adjust_period_us = 2000000U;
  float momentum_adjust_threshold_rad_s = 5.0F;
  float momentum_adjust_step_rad = 0.0034906585F;  // 0.2 deg
};

struct StandupControllerInput {
  std::uint32_t now_us = 0U;
  float theta_rad = 0.0F;
  float theta_rate_rad_s = 0.0F;
  float wheel_rate_rad_s = 0.0F;
  bool valid = false;
};

struct StandupControllerOutput {
  bool valid = false;
  bool stable = false;
  StandupPhase phase = StandupPhase::kIdle;
  float theta_reference_rad = 0.0F;
  float theta_error_rad = 0.0F;
  float filtered_rate_rad_s = 0.0F;
  float target_velocity_rad_s = 0.0F;
  float velocity_error_rad_s = 0.0F;
  float velocity_integral_v = 0.0F;
  float vq_target_v = 0.0F;
  float vq_v = 0.0F;
};

class StandupController {
 public:
  explicit StandupController(
      const StandupControllerConfig& config = StandupControllerConfig{});

  bool configure(const StandupControllerConfig& config);
  void reset(const StandupControllerInput& input);
  StandupControllerOutput update(const StandupControllerInput& input);
  const StandupControllerConfig& config() const { return config_; }
  const StandupControllerOutput& output() const { return output_; }

 private:
  static bool validConfig(const StandupControllerConfig& config);
  void resetVelocityLoop();

  StandupControllerConfig config_{};
  StandupControllerOutput output_{};
  float theta_reference_rad_ = 0.0F;
  float filtered_rate_rad_s_ = 0.0F;
  float velocity_integral_v_ = 0.0F;
  float previous_velocity_error_rad_s_ = 0.0F;
  float previous_vq_v_ = 0.0F;
  std::uint32_t previous_update_us_ = 0U;
  std::uint32_t last_unstable_us_ = 0U;
  std::uint32_t last_momentum_adjust_us_ = 0U;
  int swing_rate_sign_ = 1;
  bool stable_ = false;
  bool was_balancing_ = false;
};

const char* standupPhaseName(StandupPhase phase);

}  // namespace triwhirl
