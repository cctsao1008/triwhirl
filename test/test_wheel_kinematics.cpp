#include "triwhirl/wheel_kinematics.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

constexpr float kTwoPi = 6.28318530717958647692F;
constexpr float kRadiansPerCount = kTwoPi / 4096.0F;

bool near(const float a, const float b, const float tolerance = 1.0e-3F) {
  return std::fabs(a - b) < tolerance;
}

}  // namespace

int main() {
  using triwhirl::WheelKinematics;

  WheelKinematics wheel(0.0F);
  auto state = wheel.update(4090U, 1000U);
  assert(state.initialized);
  assert(!state.velocity_valid);
  state = wheel.update(2U, 2000U);
  assert(state.velocity_valid);
  assert(state.unwrapped_count == 4098);
  assert(state.unwrapped_angle_rad > kTwoPi);
  assert(state.velocity_rad_s > 0.0F);

  wheel.reset();
  wheel.update(2U, 1000U);
  state = wheel.update(4090U, 2000U);
  assert(state.velocity_valid);
  assert(state.unwrapped_count == -6);
  assert(state.unwrapped_angle_rad < 0.0F);
  assert(state.velocity_rad_s < 0.0F);

  wheel.reset();
  wheel.update(100U, 0xFFFFFF00U);
  state = wheel.update(101U, 0x000000F4U);
  assert(state.velocity_valid);
  const float expected_velocity = kRadiansPerCount / 0.0005F;
  assert(near(state.velocity_rad_s, expected_velocity, 1.0e-2F));

  wheel.reset();
  wheel.update(10U, 1000U);
  state = wheel.update(11U, 1000U);
  assert(!state.velocity_valid);
  state = wheel.update(12U, 2000U);
  assert(state.velocity_valid);
  assert(near(state.unwrapped_angle_rad, 12.0F * kRadiansPerCount));
  assert(near(state.instantaneous_velocity_rad_s,
              2.0F * kRadiansPerCount / 0.001F,
              1.0e-2F));

  wheel.reset();
  wheel.update(0U, 1000U);
  state = wheel.update(2048U, 2000U);
  assert(!state.velocity_valid);
  const auto before_resync = state.unwrapped_count;
  state = wheel.update(2049U, 3000U);
  assert(state.velocity_valid);
  assert(state.unwrapped_count == before_resync + 1);
  assert(state.velocity_rad_s > 0.0F);

  wheel.reset();
  state = wheel.update(0x1ABCU, 1000U);
  assert(state.raw_count == 0x0ABCU);

  WheelKinematics filtered(0.01F);
  filtered.update(0U, 1000U);
  state = filtered.update(10U, 2000U);
  const float first_velocity = state.velocity_rad_s;
  assert(first_velocity > 0.0F);
  state = filtered.update(20U, 4000U);
  assert(state.velocity_rad_s > 0.0F);
  assert(state.velocity_rad_s < first_velocity);

  std::cout << "wheel kinematics tests passed\n";
  return 0;
}
