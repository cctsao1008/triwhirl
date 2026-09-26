#include "triwhirl/standup_controller.hpp"

#include <algorithm>
#include <cmath>

#include "triwhirl/upright_geometry.hpp"

namespace triwhirl {
namespace {

constexpr float kRadToDeg = 180.0F / kPi;

bool crossedZero(const float previous, const float current) {
  return (previous > 0.0F && current <= 0.0F) ||
         (previous < 0.0F && current >= 0.0F);
}

}  // namespace

StandupController::StandupController(const StandupControllerConfig& config) {
  configure(config);
}

bool StandupController::validConfig(const StandupControllerConfig& config) {
  const float values[] = {
      config.theta_reference_rad,
      config.balance_capture_rad,
      config.balance_release_rad,
      config.swing_near_rad,
      config.settling_fall_rad,
      config.pump_v_low,
      config.pump_v_high,
      config.rate_switch_rad_s,
      config.lqr_k_angle_unstable,
      config.lqr_k_rate_unstable,
      config.lqr_k_rate_recovery_unstable,
      config.lqr_k_wheel_unstable,
      config.lqr_k_angle_stable,
      config.lqr_k_rate_stable,
      config.lqr_k_wheel_stable,
      config.gyro_rate_limit_rad_s,
      config.velocity_p_unstable,
      config.velocity_i_unstable,
      config.velocity_p_recovery_unstable,
      config.velocity_i_recovery_unstable,
      config.recovery_rate_damping_v_per_rad_s,
      config.recovery_rate_damping_limit_v,
      config.velocity_p_stable,
      config.velocity_i_stable,
      config.velocity_target_limit_rad_s,
      config.vq_limit_v,
      config.velocity_output_ramp_v_s,
      config.stable_angle_rad,
      config.momentum_adjust_threshold_rad_s,
      config.momentum_adjust_step_rad,
  };
  for (const float value : values) {
    if (!std::isfinite(value)) return false;
  }
  return config.balance_capture_rad > 0.0F &&
         config.balance_capture_rad < config.balance_release_rad &&
         config.balance_release_rad < config.swing_near_rad &&
         config.swing_near_rad < config.settling_fall_rad &&
         config.settling_fall_rad < kUprightHalfPeriodRad &&
         config.pump_v_low > 0.0F &&
         config.pump_v_low <= config.pump_v_high &&
         config.rate_switch_rad_s >= 0.0F &&
         config.gyro_rate_limit_rad_s > 0.0F &&
         config.velocity_p_unstable >= 0.0F &&
         config.velocity_i_unstable >= 0.0F &&
         config.velocity_p_recovery_unstable >= 0.0F &&
         config.velocity_i_recovery_unstable >= 0.0F &&
         config.recovery_rate_damping_v_per_rad_s >= 0.0F &&
         config.recovery_rate_damping_limit_v >= 0.0F &&
         config.velocity_p_stable >= 0.0F &&
         config.velocity_i_stable >= 0.0F &&
         config.velocity_target_limit_rad_s > 0.0F && config.vq_limit_v > 0.0F &&
         config.recovery_rate_damping_limit_v <= config.vq_limit_v &&
         config.velocity_output_ramp_v_s > 0.0F &&
         config.stable_angle_rad > 0.0F &&
         config.stable_angle_rad <= config.balance_capture_rad &&
         config.stable_delay_us > 0U && config.momentum_adjust_period_us > 0U &&
         config.momentum_adjust_threshold_rad_s >= 0.0F &&
         config.momentum_adjust_step_rad >= 0.0F;
}

bool StandupController::configure(const StandupControllerConfig& config) {
  if (!validConfig(config)) return false;
  config_ = config;
  theta_reference_rad_ = config.theta_reference_rad;
  output_ = {};
  output_.theta_reference_rad = theta_reference_rad_;
  filtered_rate_rad_s_ = 0.0F;
  resetVelocityLoop();
  previous_balance_error_rad_ = 0.0F;
  previous_update_us_ = 0U;
  last_unstable_us_ = 0U;
  last_momentum_adjust_us_ = 0U;
  swing_rate_sign_ = 1;
  stable_ = false;
  was_balancing_ = false;
  capture_crossed_upright_ = false;
  return true;
}

void StandupController::resetVelocityLoop() {
  velocity_integral_v_ = 0.0F;
  previous_velocity_error_rad_s_ = 0.0F;
  previous_vq_v_ = 0.0F;
}

void StandupController::reset(const StandupControllerInput& input) {
  theta_reference_rad_ = config_.theta_reference_rad;
  output_ = {};
  output_.theta_reference_rad = theta_reference_rad_;
  filtered_rate_rad_s_ = 0.0F;
  resetVelocityLoop();
  previous_balance_error_rad_ = 0.0F;
  previous_update_us_ = input.now_us;
  last_unstable_us_ = input.now_us;
  last_momentum_adjust_us_ = input.now_us;
  swing_rate_sign_ = input.theta_rate_rad_s < 0.0F ? -1 : 1;
  stable_ = false;
  was_balancing_ = false;
  capture_crossed_upright_ = false;
}

StandupControllerOutput StandupController::update(
    const StandupControllerInput& input) {
  StandupControllerOutput output{};
  output.theta_reference_rad = theta_reference_rad_;
  if (!input.valid || !std::isfinite(input.theta_rad) ||
      !std::isfinite(input.theta_rate_rad_s) ||
      !std::isfinite(input.wheel_rate_rad_s)) {
    output_ = output;
    return output_;
  }

  const std::uint32_t elapsed_us =
      previous_update_us_ == 0U ? 0U : input.now_us - previous_update_us_;
  const float dt_s = static_cast<float>(elapsed_us) * 1.0e-6F;
  previous_update_us_ = input.now_us;

  // The upright reference is fixed for the whole run. "stable" is a status
  // qualification only; it must never recenter the equilibrium.
  const float error_rad =
      periodicUprightErrorRad(input.theta_rad, theta_reference_rad_);
  if (!std::isfinite(error_rad)) {
    output_ = output;
    return output_;
  }
  const float abs_error = std::fabs(error_rad);

  // Before the first crossing/reversal, preserve the seller-style 9/12-degree
  // capture hysteresis so a bad approach can immediately return to swing-up.
  // After settling is latched, keep balancing all the way to the explicit
  // true-fall guard.
  const float hold_limit_rad = capture_crossed_upright_
                                   ? config_.settling_fall_rad
                                   : config_.balance_release_rad;
  const bool hold_balance = was_balancing_ && abs_error < hold_limit_rad;
  if (!hold_balance && abs_error >= config_.balance_capture_rad) {
    if (std::fabs(input.theta_rate_rad_s) >= config_.rate_switch_rad_s) {
      swing_rate_sign_ = input.theta_rate_rad_s < 0.0F ? -1 : 1;
    }
    const bool near = abs_error < config_.swing_near_rad;
    output.phase = near ? StandupPhase::kSwingLow : StandupPhase::kSwingHigh;
    output.vq_v = static_cast<float>(swing_rate_sign_) *
                  (near ? config_.pump_v_low : config_.pump_v_high);
    output.target_velocity_rad_s = 0.0F;
    output.theta_error_rad = error_rad;
    output.theta_reference_rad = theta_reference_rad_;
    output.stable = false;
    output.settling = false;
    output.valid = true;

    filtered_rate_rad_s_ = 0.0F;
    previous_balance_error_rad_ = 0.0F;
    capture_crossed_upright_ = false;
    was_balancing_ = false;
    stable_ = false;
    last_unstable_us_ = input.now_us;
    output_ = output;
    return output_;
  }

  bool just_latched_settling = false;
  const bool entering_balance = !was_balancing_;
  if (entering_balance) {
    resetVelocityLoop();
    capture_crossed_upright_ = false;
    previous_balance_error_rad_ = error_rad;
    stable_ = false;
    last_unstable_us_ = input.now_us;
  } else if (!capture_crossed_upright_ &&
             crossedZero(previous_balance_error_rad_, error_rad)) {
    capture_crossed_upright_ = true;
    just_latched_settling = true;
  }
  previous_balance_error_rad_ = error_rad;

  const float vendor_rate_rad_s = std::clamp(
      input.theta_rate_rad_s, -config_.gyro_rate_limit_rad_s,
      config_.gyro_rate_limit_rad_s);
  filtered_rate_rad_s_ =
      0.6F * filtered_rate_rad_s_ + 0.4F * vendor_rate_rad_s;
  was_balancing_ = true;

  const float error_deg = error_rad * kRadToDeg;
  const float filtered_rate_deg_s = filtered_rate_rad_s_ * kRadToDeg;

  if (!capture_crossed_upright_) {
    // A low-energy approach can reverse just before crossing zero. Treat that as
    // arrival only inside the same +/-5-degree neighbourhood used for stable
    // qualification. A reversal farther out is a failed approach and must retain
    // the original 12-degree release path back to swing-up.
    const float phase_product = error_deg * filtered_rate_deg_s;
    const bool at_zero_with_motion =
        std::fabs(error_deg) < 1.0e-4F &&
        std::fabs(filtered_rate_deg_s) > 1.0e-4F;
    const bool reversal_near_upright = abs_error <= config_.stable_angle_rad;
    if (reversal_near_upright &&
        (phase_product > 0.0F || at_zero_with_motion)) {
      capture_crossed_upright_ = true;
      just_latched_settling = true;
    }
  }

  // One second inside +/-5 degrees is only a success/qualification flag. The
  // same bilateral settling law keeps running afterward, and the fixed upright
  // reference never moves. Leaving +/-5 degrees clears the flag but does not
  // release settling or stop active correction.
  if (abs_error > config_.stable_angle_rad) {
    last_unstable_us_ = input.now_us;
    stable_ = false;
  } else if (!stable_ &&
             (input.now_us - last_unstable_us_) >= config_.stable_delay_us) {
    stable_ = true;
    if (!capture_crossed_upright_) {
      capture_crossed_upright_ = true;
      just_latched_settling = true;
    }
  }

  if (just_latched_settling) {
    velocity_integral_v_ = 0.0F;
    previous_velocity_error_rad_s_ = 0.0F;
  }

  const bool settling_mode = capture_crossed_upright_;
  float target_velocity_unclamped = 0.0F;
  if (settling_mode) {
    // Bilateral local regulator: wheel target is position restoring only. Body
    // rate damping is applied independently in the Vq path below, so rate/wheel
    // feedback cannot swamp the sign reversal of the position command.
    //   error > 0 -> target < 0
    //   error < 0 -> target > 0
    target_velocity_unclamped = config_.lqr_k_angle_unstable * error_deg;
  } else {
    // First approach remains the already-proven seller-style state feedback.
    target_velocity_unclamped =
        config_.lqr_k_angle_unstable * error_deg +
        config_.lqr_k_rate_unstable * (-filtered_rate_deg_s) +
        config_.lqr_k_wheel_unstable * input.wheel_rate_rad_s;
  }
  const bool target_saturated =
      std::fabs(target_velocity_unclamped) > config_.velocity_target_limit_rad_s;
  const float target_velocity = std::clamp(
      target_velocity_unclamped, -config_.velocity_target_limit_rad_s,
      config_.velocity_target_limit_rad_s);

  const float velocity_error = target_velocity - input.wheel_rate_rad_s;
  const float kp = settling_mode ? config_.velocity_p_recovery_unstable
                                 : config_.velocity_p_unstable;
  const float ki = settling_mode ? config_.velocity_i_recovery_unstable
                                 : config_.velocity_i_unstable;
  // Empirical coordinate contract from standup-20260926-152656 near upright:
  //   theta_rate=-2.424 rad/s, Vq=+2.358 V -> next rate=-2.059 rad/s
  //   theta_rate=+2.753 rad/s, Vq=-2.374 V -> next rate=+2.313 rad/s
  // Thus Vq and body angular acceleration have the same sign in the software
  // coordinates. Dissipative damping must command Vq OPPOSITE body rate. This
  // also matches the vendor outer-rate term, which is proportional to -Gyro.
  const float direct_recovery_vq = settling_mode
      ? std::clamp(-config_.recovery_rate_damping_v_per_rad_s *
                       filtered_rate_rad_s_,
                   -config_.recovery_rate_damping_limit_v,
                   config_.recovery_rate_damping_limit_v)
      : 0.0F;

  if (dt_s > 0.0F && dt_s < 0.1F) {
    const float integral_delta =
        0.5F * ki * dt_s *
        (velocity_error + previous_velocity_error_rad_s_);
    const float candidate_integral = std::clamp(
        velocity_integral_v_ + integral_delta,
        -config_.vq_limit_v, config_.vq_limit_v);
    const float candidate_unclamped_vq =
        kp * velocity_error + candidate_integral + direct_recovery_vq;
    const bool pushes_positive_saturation =
        candidate_unclamped_vq > config_.vq_limit_v && integral_delta > 0.0F;
    const bool pushes_negative_saturation =
        candidate_unclamped_vq < -config_.vq_limit_v && integral_delta < 0.0F;
    if (!pushes_positive_saturation && !pushes_negative_saturation) {
      velocity_integral_v_ = candidate_integral;
    }
  }
  previous_velocity_error_rad_s_ = velocity_error;

  const float vq_unclamped =
      kp * velocity_error + velocity_integral_v_ + direct_recovery_vq;
  const bool vq_saturated = std::fabs(vq_unclamped) > config_.vq_limit_v;
  const float vq_target =
      std::clamp(vq_unclamped, -config_.vq_limit_v, config_.vq_limit_v);
  float vq = vq_target;
  if (dt_s > 0.0F && dt_s < 0.1F) {
    const float max_step = config_.velocity_output_ramp_v_s * dt_s;
    vq = std::clamp(vq_target, previous_vq_v_ - max_step,
                    previous_vq_v_ + max_step);
  }
  previous_vq_v_ = vq;

  output.phase = StandupPhase::kBalance;
  output.theta_error_rad = error_rad;
  output.theta_reference_rad = theta_reference_rad_;
  output.filtered_rate_rad_s = filtered_rate_rad_s_;
  output.target_velocity_rad_s = target_velocity;
  output.velocity_error_rad_s = velocity_error;
  output.velocity_integral_v = velocity_integral_v_;
  output.vq_unclamped_v = vq_unclamped;
  output.vq_target_v = vq_target;
  output.vq_v = vq;
  output.stable = stable_;
  output.settling = settling_mode;
  output.target_saturated = target_saturated;
  output.vq_saturated = vq_saturated;
  output.valid = std::isfinite(vq) && std::isfinite(target_velocity) &&
                 std::isfinite(output.theta_reference_rad) &&
                 std::isfinite(output.theta_error_rad);
  output_ = output;
  return output_;
}

const char* standupPhaseName(const StandupPhase phase) {
  switch (phase) {
    case StandupPhase::kIdle: return "idle";
    case StandupPhase::kSwingHigh: return "swing_high";
    case StandupPhase::kSwingLow: return "swing_low";
    case StandupPhase::kBalance: return "balance";
  }
  return "unknown";
}

}  // namespace triwhirl
