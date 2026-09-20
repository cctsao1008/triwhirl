#pragma once

#include <cmath>

namespace triwhirl {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kUprightPeriodDeg = 120.0F;
constexpr float kUprightHalfPeriodDeg = 60.0F;
constexpr float kUprightPeriodRad = 2.0F * kPi / 3.0F;
constexpr float kUprightHalfPeriodRad = kPi / 3.0F;

inline float wrapPeriodic(float value, const float period,
                          const float half_period) {
  if (!std::isfinite(value) || !std::isfinite(period) ||
      !std::isfinite(half_period) || period <= 0.0F) {
    return NAN;
  }
  float wrapped = std::fmod(value + half_period, period);
  if (wrapped < 0.0F) {
    wrapped += period;
  }
  return wrapped - half_period;
}

// Shared balance coordinate for all three Reuleaux upright vertices.
// A/B/C are separated by 120 degrees, so theta, theta +/- 120 deg, ... map
// to the same local equilibrium coordinate in [-60, 60) degrees.
inline float periodicUprightErrorDeg(const float theta_deg,
                                     const float reference_deg) {
  return wrapPeriodic(theta_deg - reference_deg, kUprightPeriodDeg,
                      kUprightHalfPeriodDeg);
}

// Radian form used by the real-time balance controller.  The result is in
// [-pi/3, pi/3), so one controller can operate at any physical upright vertex.
inline float periodicUprightErrorRad(const float theta_rad,
                                     const float reference_rad) {
  return wrapPeriodic(theta_rad - reference_rad, kUprightPeriodRad,
                      kUprightHalfPeriodRad);
}

}  // namespace triwhirl
