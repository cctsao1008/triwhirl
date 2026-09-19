#include "triwhirl/wheel_kinematics.hpp"

#include <cmath>

namespace triwhirl {
namespace {
constexpr float kTwoPi = 6.28318530717958647692F;
constexpr float kMicrosToSeconds = 1.0e-6F;
constexpr float kRadiansPerCount = kTwoPi / 4096.0F;
}  // namespace

WheelKinematics::WheelKinematics(const float velocity_time_constant_s)
    : velocity_time_constant_s_((std::isfinite(velocity_time_constant_s) &&
                                 velocity_time_constant_s > 0.0F)
                                    ? velocity_time_constant_s
                                    : 0.0F) {}

void WheelKinematics::reset() {
  previous_raw_count_ = 0;
  previous_timestamp_us_ = 0;
  accumulated_counts_ = 0;
  state_ = {};
}

WheelKinematicsState WheelKinematics::update(const std::uint16_t raw_count,
                                             const std::uint32_t timestamp_us) {
  const std::uint16_t count = raw_count & 0x0FFFU;
  state_.raw_count = count;
  state_.angle_rad = static_cast<float>(count) * kRadiansPerCount;
  if (!state_.initialized) {
    previous_raw_count_ = count;
    previous_timestamp_us_ = timestamp_us;
    accumulated_counts_ = count;
    state_.unwrapped_count = accumulated_counts_;
    state_.unwrapped_angle_rad = state_.angle_rad;
    state_.velocity_rad_s = 0.0F;
    state_.instantaneous_velocity_rad_s = 0.0F;
    state_.initialized = true;
    state_.velocity_valid = false;
    return state_;
  }

  const std::uint32_t elapsed_us = timestamp_us - previous_timestamp_us_;
  std::int32_t delta_counts = static_cast<std::int32_t>(count) -
                              static_cast<std::int32_t>(previous_raw_count_);
  if (delta_counts > kHalfTurnCounts) {
    delta_counts -= static_cast<std::int32_t>(kCountsPerTurn);
  } else if (delta_counts < -kHalfTurnCounts) {
    delta_counts += static_cast<std::int32_t>(kCountsPerTurn);
  }

  if (delta_counts == kHalfTurnCounts || delta_counts == -kHalfTurnCounts) {
    previous_raw_count_ = count;
    previous_timestamp_us_ = timestamp_us;
    state_.velocity_valid = false;
    state_.instantaneous_velocity_rad_s = 0.0F;
    return state_;
  }
  if (elapsed_us == 0U) {
    state_.velocity_valid = false;
    state_.instantaneous_velocity_rad_s = 0.0F;
    return state_;
  }

  previous_raw_count_ = count;
  previous_timestamp_us_ = timestamp_us;
  accumulated_counts_ += delta_counts;
  state_.unwrapped_count = accumulated_counts_;
  state_.unwrapped_angle_rad =
      static_cast<float>(accumulated_counts_) * kRadiansPerCount;
  const float dt_s = static_cast<float>(elapsed_us) * kMicrosToSeconds;
  const float instantaneous_velocity =
      static_cast<float>(delta_counts) * kRadiansPerCount / dt_s;
  state_.instantaneous_velocity_rad_s = instantaneous_velocity;
  if (!state_.velocity_valid || velocity_time_constant_s_ <= 0.0F) {
    state_.velocity_rad_s = instantaneous_velocity;
  } else {
    const float alpha = dt_s / (velocity_time_constant_s_ + dt_s);
    state_.velocity_rad_s +=
        alpha * (instantaneous_velocity - state_.velocity_rad_s);
  }
  state_.velocity_valid = true;
  return state_;
}

}  // namespace triwhirl
