#pragma once

#include <cstdint>

namespace triwhirl {

struct WheelKinematicsState {
  std::uint16_t raw_count = 0;
  float angle_rad = 0.0F;
  std::int64_t unwrapped_count = 0;
  float unwrapped_angle_rad = 0.0F;
  float velocity_rad_s = 0.0F;
  float instantaneous_velocity_rad_s = 0.0F;
  bool initialized = false;
  bool velocity_valid = false;
};

class WheelKinematics {
 public:
  explicit WheelKinematics(float velocity_time_constant_s = 0.01F);
  void reset();
  WheelKinematicsState update(std::uint16_t raw_count,
                              std::uint32_t timestamp_us);
  const WheelKinematicsState& state() const { return state_; }

 private:
  static constexpr std::uint16_t kCountsPerTurn = 4096U;
  static constexpr std::int32_t kHalfTurnCounts = 2048;
  float velocity_time_constant_s_;
  std::uint16_t previous_raw_count_ = 0;
  std::uint32_t previous_timestamp_us_ = 0;
  std::int64_t accumulated_counts_ = 0;
  WheelKinematicsState state_{};
};

}  // namespace triwhirl
