#include "triwhirl/attitude_estimator.hpp"

#include <cmath>

namespace triwhirl {
namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 6.28318530717958647692F;

float wrapAngle(float angle_rad) {
  if (!std::isfinite(angle_rad)) {
    return 0.0F;
  }

  // Normal estimator updates move by only a small fraction of one revolution.
  // Keep that hot path out of fmod(), which is comparatively expensive on the
  // ESP32 LX6. Retain a general fallback for explicit resets with large angles.
  if (angle_rad > 3.0F * kTwoPi || angle_rad < -3.0F * kTwoPi) {
    angle_rad = std::fmod(angle_rad + kPi, kTwoPi);
    if (angle_rad < 0.0F) {
      angle_rad += kTwoPi;
    }
    return angle_rad - kPi;
  }
  while (angle_rad >= kPi) {
    angle_rad -= kTwoPi;
  }
  while (angle_rad < -kPi) {
    angle_rad += kTwoPi;
  }
  return angle_rad;
}

float clamp01(const float value) {
  return value < 0.0F ? 0.0F : (value > 1.0F ? 1.0F : value);
}

inline void fastSinCos(const float angle_rad, float* const sin_out,
                       float* const cos_out) {
  // GCC can lower this pair to the target's combined sincosf implementation,
  // avoiding two independent range reductions/libm calls on every estimator
  // sample. Keep this local so numerical semantics stay identical to sin/cos.
  __builtin_sincosf(angle_rad, sin_out, cos_out);
}

}  // namespace

PlanarAttitudeEstimator::PlanarAttitudeEstimator(
    const AttitudeEstimatorConfig& config)
    : config_(config) {}

void PlanarAttitudeEstimator::reset(const float angle_rad,
                                    const float gyro_bias_rad_s) {
  state_ = {};
  state_.angle_rad = wrapAngle(angle_rad);
  state_.gyro_bias_rad_s =
      std::isfinite(gyro_bias_rad_s) ? gyro_bias_rad_s : 0.0F;
}

AttitudeEstimate PlanarAttitudeEstimator::update(
    const float body_accel_x_mps2,
    const float body_accel_z_mps2,
    const float body_gyro_rad_s,
    const float dt_s,
    const bool allow_bias_update) {
  if (!std::isfinite(body_accel_x_mps2) ||
      !std::isfinite(body_accel_z_mps2) ||
      !std::isfinite(body_gyro_rad_s) || !std::isfinite(dt_s) ||
      !(dt_s > 0.0F) || dt_s > 0.1F) {
    state_.valid = false;
    return state_;
  }

  // Only a two-axis Euclidean norm is needed here. sqrt(x*x+z*z) avoids the
  // extra generality/edge-case machinery in hypot() on every control sample.
  const float accel_norm_sq =
      body_accel_x_mps2 * body_accel_x_mps2 +
      body_accel_z_mps2 * body_accel_z_mps2;
  const float accel_norm = accel_norm_sq > 0.0F ? std::sqrt(accel_norm_sq) : 0.0F;
  float accel_weight = 0.0F;
  float innovation = 0.0F;

  if (accel_norm > 1.0e-5F) {
    const float ax = body_accel_x_mps2 / accel_norm;
    const float az = body_accel_z_mps2 / accel_norm;
    float gravity_x = 0.0F;
    float gravity_z = 1.0F;
    fastSinCos(state_.angle_rad, &gravity_x, &gravity_z);

    // innovation = sin(theta_est - theta_accel).  Positive innovation means
    // the estimate is ahead of gravity, so both proportional correction and
    // bias adaptation must act in the negative-error direction.
    innovation = gravity_x * az - gravity_z * ax;

    if (accel_norm >= config_.accel_norm_min_mps2 &&
        accel_norm <= config_.accel_norm_max_mps2) {
      constexpr float kGravity = 9.80665F;
      const float half_span =
          0.5F * (config_.accel_norm_max_mps2 - config_.accel_norm_min_mps2);
      if (half_span > 1.0e-5F) {
        accel_weight =
            clamp01(1.0F - std::fabs(accel_norm - kGravity) / half_span);
      } else {
        accel_weight = 1.0F;
      }
    }
  }

  if (allow_bias_update && accel_weight > 0.0F) {
    state_.gyro_bias_rad_s +=
        config_.ki * accel_weight * innovation * dt_s;
  }

  state_.rate_rad_s = body_gyro_rad_s - state_.gyro_bias_rad_s;
  const float corrected_rate =
      state_.rate_rad_s - config_.kp * accel_weight * innovation;
  state_.angle_rad = wrapAngle(state_.angle_rad + corrected_rate * dt_s);
  state_.gravity_innovation = innovation;
  state_.accel_weight = accel_weight;
  state_.valid = true;
  return state_;
}

}  // namespace triwhirl
