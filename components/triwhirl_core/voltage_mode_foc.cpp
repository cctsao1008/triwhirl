#include "triwhirl/voltage_mode_foc.hpp"

#include <algorithm>
#include <cmath>

namespace triwhirl {
namespace {
constexpr float kTwoPi = 6.28318530717958647692F;
constexpr float kInvSqrt3 = 0.5773502691896258F;
constexpr float kSqrt3Over2 = 0.8660254037844386F;

float clampValue(const float value, const float low, const float high) {
  return value < low ? low : (value > high ? high : value);
}

inline void fastSinCos(const float angle_rad, float* const sin_out,
                       float* const cos_out) {
  // Use one combined libm operation so the electrical-angle range reduction is
  // not repeated for separate sin() and cos() calls on every active FOC update.
  __builtin_sincosf(angle_rad, sin_out, cos_out);
}
}  // namespace

bool validMotorElectricalConfig(const MotorElectricalConfig& config) {
  return config.pole_pairs > 0 && config.pole_pairs <= 64 &&
         (config.sensor_direction == 1 || config.sensor_direction == -1) &&
         std::isfinite(config.electrical_offset_rad);
}

float wrapElectricalAngle(float angle_rad) {
  if (!std::isfinite(angle_rad)) {
    return 0.0F;
  }

  // Normal FOC updates arrive close to the previous electrical angle. Keep the
  // common case out of fmod(), which is expensive on ESP32 LX6.
  if (angle_rad >= 0.0F && angle_rad < kTwoPi) {
    return angle_rad;
  }
  if (angle_rad > 8.0F * kTwoPi || angle_rad < -8.0F * kTwoPi) {
    angle_rad = std::fmod(angle_rad, kTwoPi);
  }
  while (angle_rad >= kTwoPi) {
    angle_rad -= kTwoPi;
  }
  while (angle_rad < 0.0F) {
    angle_rad += kTwoPi;
  }
  return angle_rad;
}

float electricalAngleFromMechanical(const float mechanical_angle_rad,
                                    const MotorElectricalConfig& config) {
  if (!validMotorElectricalConfig(config) ||
      !std::isfinite(mechanical_angle_rad)) {
    return 0.0F;
  }
  return wrapElectricalAngle(
      static_cast<float>(config.sensor_direction * config.pole_pairs) *
          mechanical_angle_rad +
      config.electrical_offset_rad);
}

PhaseVoltages makeDqVoltage(const float electrical_angle_rad,
                            const float vd_v,
                            const float vq_v,
                            const float bus_voltage_v,
                            const float vector_limit_v) {
  if (!(bus_voltage_v > 0.0F) || !std::isfinite(bus_voltage_v) ||
      !(vector_limit_v > 0.0F) || !std::isfinite(vector_limit_v) ||
      !std::isfinite(vd_v) || !std::isfinite(vq_v) ||
      !std::isfinite(electrical_angle_rad)) {
    return {0.0F, 0.0F, 0.0F};
  }

  float vd = vd_v;
  float vq = vq_v;
  const float linear_limit = std::min(vector_limit_v, bus_voltage_v * kInvSqrt3);
  const float requested_sq = vd * vd + vq * vq;
  const float linear_limit_sq = linear_limit * linear_limit;
  if (requested_sq > linear_limit_sq && requested_sq > 0.0F) {
    // sqrt() is needed only on the saturation path. Normal TriWhirl bring-up
    // Vq commands are already within the configured vector limit.
    const float scale = linear_limit / std::sqrt(requested_sq);
    vd *= scale;
    vq *= scale;
  }

  // electricalAngleFromMechanical() already returns [0, 2*pi) in the normal
  // FOC path; wrapElectricalAngle() therefore takes its one-compare fast path.
  const float angle = wrapElectricalAngle(electrical_angle_rad);
  float sin_theta = 0.0F;
  float cos_theta = 1.0F;
  fastSinCos(angle, &sin_theta, &cos_theta);

  const float alpha = vd * cos_theta - vq * sin_theta;
  const float beta = vd * sin_theta + vq * cos_theta;

  const float phase_a = alpha;
  const float phase_b = -0.5F * alpha + kSqrt3Over2 * beta;
  const float phase_c = -0.5F * alpha - kSqrt3Over2 * beta;

  const float phase_max = std::max(phase_a, std::max(phase_b, phase_c));
  const float phase_min = std::min(phase_a, std::min(phase_b, phase_c));
  const float common_mode = -0.5F * (phase_max + phase_min);
  const float center = 0.5F * bus_voltage_v;

  return {
      clampValue(center + phase_a + common_mode, 0.0F, bus_voltage_v),
      clampValue(center + phase_b + common_mode, 0.0F, bus_voltage_v),
      clampValue(center + phase_c + common_mode, 0.0F, bus_voltage_v),
  };
}

}  // namespace triwhirl
