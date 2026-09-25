#include "triwhirl/standup_controller.hpp"

#include <algorithm>
#include <cmath>

#include "triwhirl/upright_geometry.hpp"

namespace triwhirl {
namespace {

constexpr float kRadToDeg = 180.0F / kPi;

float signOr(const float value, const int fallback) {
  if (value > 0.0F) return 1.0F;
  if (value < 0.0F) return -1.0F;
  return fallback >= 0 ? 1.0F : -1.0F;
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
         config.swing_near_rad <= kUprightHalfPeriodRad &&
         config.pump_v_low > 0.0F &&
         config.pump_v_low <= config.pump_v_high &&
         config.rate_switch_rad_s >= 0.0F &&
         config.gyro_rate_limit_rad_s > 0.0F &&
         config.velocity_p_unstable >= 0.0F &&
         config.velocity_i_unstable >= 0.0F &&
         config.velocity_p_stable >= 0.0F &&
         config.velocity_i_stable >= 0.0F &&
         config.velocity_target_limit_rad_s > 0.0F && config.vq_limit_v > 0.0F &&
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
  previous_update_us_ = 0U;
  last_unstable_us_ = 0U;
  last_momentum_adjust_us_ = 0U;
  swing_rate_sign_ = 1;
  stable_ = false;
  was_balancing_ = false;
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
  previous_update_us_ = input.now_us;
  last_unstable_us_ = input.now_us;
  last_momentum_adjust_us_ = input.now_us;
  swing_rate_sign_ = input.theta_rate_rad_s < 0.0F ? -1 : 1;
  stable_ = false;
  was_balancing_ = false;
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

  const float error_rad =
      periodicUprightErrorRad(input.theta_rad, theta_reference_rad_);
  if (!std::isfinite(error_rad)) {
    output_ = output;
    return output_;
  }
  const float abs_error = std::fabs(error_rad);

  const bool hold_balance =
      was_balancing_ && abs_error < config_.balance_release_rad;
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
    output.valid = true;

    filtered_rate_rad_s_ = 0.0F;
    was_balancing_ = false;
    output_ = output;
    return output_;
  }

  const bool entering_balance = !was_balancing_;
  if (entering_balance) {
    // Each capture attempt starts with a clean inner-loop state. The lossless
    // 2026-09-23 trace showed failed captures leaving the integrator at -4 V,
    // which made subsequent captures begin fully saturated.
    resetVelocityLoop();
  }

  const float vendor_rate_rad_s = std::clamp(
      input.theta_rate_rad_s, -config_.gyro_rate_limit_rad_s,
      config_.gyro_rate_limit_rad_s);
  filtered_rate_rad_s_ =
      0.6F * filtered_rate_rad_s_ + 0.4F * vendor_rate_rad_s;
  was_balancing_ = true;

  if (abs_error > config_.stable_angle_rad) {
    last_unstable_us_ = input.now_us;
    if (stable_) {
      theta_reference_rad_ = config_.theta_reference_rad;
      stable_ = false;
    }
  }
  if (!stable_ &&
      (input.now_us - last_unstable_us_) > config_.stable_delay_us) {
    theta_reference_rad_ += error_rad;
    stable_ = true;
  }

  const float error_deg = error_rad * kRadToDeg;
  const float filtered_rate_deg_s = filtered_rate_rad_s_ * kRadToDeg;
  const float k_angle = stable_ ? config_.lqr_k_angle_stable
                                : config_.lqr_k_angle_unstable;
  float k_rate = stable_ ? config_.lqr_k_rate_stable
                         : config_.lqr_k_rate_unstable;
  if (!stable_) {
    // e * de/dt >= 0 means the body is moving away from the upright (or is
    // crossing it with finite rate). Use stronger damping only in that half of
    // phase space; retain the lower gain while approaching so capture energy is
    // not removed before the body reaches zero error.
    const bool moving_away_or_crossing = error_deg * filtered_rate_deg_s >= 0.0F;
    if (moving_away_or_crossing) {
      k_rate = config_.lqr_k_rate_recovery_unstable;
    }
  }
  const float k_wheel = stable_ ? config_.lqr_k_wheel_stable
                                : config_.lqr_k_wheel_unstable;

  float target_velocity =
      k_angle * error_deg + k_rate * (-filtered_rate_deg_s) +
      k_wheel * input.wheel_rate_rad_s;
  target_velocity = std::clamp(target_velocity,
                               -config_.velocity_target_limit_rad_s,
                               config_.velocity_target_limit_rad_s);

  if (stable_ &&
      (input.now_us - last_momentum_adjust_us_) >=
          config_.momentum_adjust_period_us &&
      std::fabs(target_velocity) > config_.momentum_adjust_threshold_rad_s) {
    theta_reference_rad_ +=
        signOr(target_velocity, 1) * config_.momentum_adjust_step_rad;
    last_momentum_adjust_us_ = input.now_us;
  }

  const float velocity_error = target_velocity - input.wheel_rate_rad_s;
  const float kp = stable_ ? config_.velocity_p_stable
                           : config_.velocity_p_unstable;
  const float ki = stable_ ? config_.velocity_i_stable
                           : config_.velocity_i_unstable;
  if (dt_s > 0.0F && dt_s < 0.1F) {
    const float integral_delta =
        0.5F * ki * dt_s *
        (velocity_error + previous_velocity_error_rad_s_);
    const float candidate_integral = std::clamp(
        velocity_integral_v_ + integral_delta,
        -config_.vq_limit_v, config_.vq_limit_v);
    const float candidate_unclamped_vq =
        kp * velocity_error + candidate_integral;
    const bool pushes_positive_saturation =
        candidate_unclamped_vq > config_.vq_limit_v && integral_delta > 0.0F;
    const bool pushes_negative_saturation =
        candidate_unclamped_vq < -config_.vq_limit_v && integral_delta < 0.0F;
    if (!pushes_positive_saturation && !pushes_negative_saturation) {
      velocity_integral_v_ = candidate_integral;
    }
  }
  previous_velocity_error_rad_s_ = velocity_error;

  const float vq_target =
      std::clamp(kp * velocity_error + velocity_integral_v_,
                 -config_.vq_limit_v, config_.vq_limit_v);
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
  output.vq_target_v = vq_target;
  output.vq_v = vq;
  output.stable = stable_;
  output.valid = std::isfinite(vq) && std::isfinite(target_velocity) &&
                 std::isfinite(output.theta_reference_rad) &&
                 std::isfinite(output.theta_error_rad);
  output_ = output;
  return output_;
}

const char* standupPhaseName(StandupPhase phase) {
  switch (phase) {
    case StandupPhase::kIdle: return "idle";
    case StandupPhase::kSwingHigh: return "swing_high";
    case StandupPhase::kSwingLow: return "swing_low";
    case StandupPhase::kBalance: return "balance";
  }
  return "unknown";
}

}  // namespace triwhirl
