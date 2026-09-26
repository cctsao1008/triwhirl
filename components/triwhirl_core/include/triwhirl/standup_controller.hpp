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

  // Keep the proven TRC-V1.1 swing/balance handoff envelope: enter Balance at
  // 9 deg, hold it until 12 deg before settling is acquired, and use the lower
  // swing voltage inside 18 deg. Once settling is acquired, keep correcting
  // until the body reaches the explicit true-fall guard near the +/-60-deg
  // periodic-coordinate boundary.
  float balance_capture_rad = 0.1570796327F;  // 9 deg
  float balance_release_rad = 0.2094395102F;  // 12 deg
  float swing_near_rad = 0.3141592654F;       // 18 deg
  float settling_fall_rad = 0.9599310886F;    // 55 deg
  float pump_v_low = 0.168F;
  float pump_v_high = 0.42F;
  float rate_switch_rad_s = 0.03F;

  // Trace-tuned commissioning gains. The first approach keeps the vendor-style
  // outer state feedback -> reaction-wheel velocity target -> velocity PI path.
  // After upright acquisition, the settling target is bilateral position
  // restoring and body-rate damping is applied independently in the Vq path.
  // Angle/rate are in degrees(/s); wheel and target velocity are in rad/s.
  float lqr_k_angle_unstable = -8.0F;
  float lqr_k_rate_unstable = 0.35F;
  float lqr_k_rate_recovery_unstable = 0.55F;  // retained legacy tuning field
  float lqr_k_wheel_unstable = 0.30F;
  float lqr_k_angle_stable = -2.5F;            // retained legacy tuning field
  float lqr_k_rate_stable = 0.35F;             // retained legacy tuning field
  float lqr_k_wheel_stable = 0.20F;            // retained legacy tuning field

  // Golden TRC-V1.1 configures the MPU6050 gyro for +/-250 deg/s. Our runtime
  // deliberately uses a wider sensor range, so clamp only the standup-controller
  // input to that envelope before applying the retained 0.6/0.4 gyro filter.
  float gyro_rate_limit_rad_s = 4.36332313F;  // 250 deg/s

  // Approach and settling use separate wheel-velocity PI settings. The runtime
  // currently uses the same P on both paths, no settling integral, plus bounded
  // direct body-rate damping in the Vq path.
  float velocity_p_unstable = 0.035F;
  float velocity_i_unstable = 0.150F;
  float velocity_p_recovery_unstable = 0.100F;
  float velocity_i_recovery_unstable = 0.0F;
  float recovery_rate_damping_v_per_rad_s = 0.0F;
  float recovery_rate_damping_limit_v = 0.0F;
  float velocity_p_stable = 0.018F;  // retained legacy tuning field
  float velocity_i_stable = 0.100F;  // retained legacy tuning field
  float velocity_target_limit_rad_s = 60.0F;
  float vq_limit_v = 3.0F;
  // Preserve the Arduino-FOC 2.1.1 velocity-PI slew limit.
  float velocity_output_ramp_v_s = 1000.0F;

  // One second inside +/-5 degrees is a qualification/status indication only.
  // It does not recenter the reference or stop bilateral correction.
  float stable_angle_rad = 0.0872664626F;  // 5 deg
  std::uint32_t stable_delay_us = 1000000U;

  // Retained for future momentum-unloading work; not used by the current
  // commissioning settling law.
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
  bool settling = false;
  bool target_saturated = false;
  bool vq_saturated = false;
  StandupPhase phase = StandupPhase::kIdle;
  float theta_reference_rad = 0.0F;
  float theta_error_rad = 0.0F;
  float filtered_rate_rad_s = 0.0F;
  float target_velocity_rad_s = 0.0F;
  float velocity_error_rad_s = 0.0F;
  float velocity_integral_v = 0.0F;
  float vq_unclamped_v = 0.0F;
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
  float previous_balance_error_rad_ = 0.0F;
  std::uint32_t previous_update_us_ = 0U;
  std::uint32_t last_unstable_us_ = 0U;
  std::uint32_t last_momentum_adjust_us_ = 0U;
  int swing_rate_sign_ = 1;
  bool stable_ = false;
  bool was_balancing_ = false;
  bool capture_crossed_upright_ = false;
};

const char* standupPhaseName(StandupPhase phase);

}  // namespace triwhirl