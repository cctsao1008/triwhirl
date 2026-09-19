#include "triwhirl/three_phase_field.hpp"

#include <cmath>

namespace triwhirl {
namespace {
constexpr float kSqrt3Over2 = 0.8660254037844386F;
float clampValue(const float value, const float low, const float high) {
  return value < low ? low : (value > high ? high : value);
}
}  // namespace

PhaseVoltages makeRotatingField(const float electrical_angle_rad,
                                const float amplitude_v,
                                const float voltage_limit_v) {
  if (!(voltage_limit_v > 0.0F) || !std::isfinite(voltage_limit_v) ||
      !std::isfinite(amplitude_v) || !std::isfinite(electrical_angle_rad)) {
    return {0.0F, 0.0F, 0.0F};
  }
  const float max_amplitude = 0.5F * voltage_limit_v;
  const float amplitude = clampValue(amplitude_v, 0.0F, max_amplitude);
  const float center = 0.5F * voltage_limit_v;
  const float alpha = -amplitude * std::sin(electrical_angle_rad);
  const float beta = amplitude * std::cos(electrical_angle_rad);
  return {
      clampValue(alpha + center, 0.0F, voltage_limit_v),
      clampValue((-0.5F * alpha + kSqrt3Over2 * beta) + center, 0.0F,
                 voltage_limit_v),
      clampValue((-0.5F * alpha - kSqrt3Over2 * beta) + center, 0.0F,
                 voltage_limit_v),
  };
}

}  // namespace triwhirl
