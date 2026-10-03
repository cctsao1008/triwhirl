#pragma once

#include <array>
#include <cmath>
#include <cstddef>

namespace triwhirl::sitl {

// Provisional near-upright linear plant used by the existing B/C SITL evidence.
// The coefficients remain in their identified Vq coordinate; this helper adds
// a mathematically equivalent wheel-acceleration coordinate for simulation only.
// It does not turn the real motor into an ideal acceleration source.
struct LocalLinearPlant {
  const char* name = nullptr;
  std::array<std::array<double, 3>, 3> a{};
  std::array<double, 3> b{};
};

struct LocalLinearState {
  double theta_error_rad = 0.0;
  double theta_rate_rad_s = 0.0;
  double wheel_rate_rad_s = 0.0;
};

struct LocalLinearDerivative {
  bool valid = false;
  double theta_error_rate_rad_s = 0.0;
  double theta_accel_rad_s2 = 0.0;
  double wheel_accel_rad_s2 = 0.0;
};

struct WheelAccelerationCoordinateResult {
  bool valid = false;
  // Latent algebraic coordinate used only to preserve the identified local
  // model. This is not a production actuator command or fuzzy-controller output.
  double equivalent_vq_v = 0.0;
  LocalLinearDerivative derivative{};
};

constexpr double kMinWheelVqGainRadS2PerV = 1.0e-12;

inline bool finiteState(const LocalLinearState& state) {
  return std::isfinite(state.theta_error_rad) &&
         std::isfinite(state.theta_rate_rad_s) &&
         std::isfinite(state.wheel_rate_rad_s);
}

inline bool validLocalLinearPlant(const LocalLinearPlant& plant) {
  if (plant.name == nullptr || plant.name[0] == '\0') return false;
  for (const auto& row : plant.a) {
    for (const double value : row) {
      if (!std::isfinite(value)) return false;
    }
  }
  for (const double value : plant.b) {
    if (!std::isfinite(value)) return false;
  }
  return std::fabs(plant.b[2]) > kMinWheelVqGainRadS2PerV;
}

inline LocalLinearPlant provisionalLocalB() {
  LocalLinearPlant plant{};
  plant.name = "swing-native-04-B-current-vq";
  plant.a = {{{0.0, 1.0, 0.0},
              {149.234, 4.205, -0.871},
              {-111.267, 11.347, -6.451}}};
  plant.b = {{0.0, 5.099, 181.134}};
  return plant;
}

inline LocalLinearPlant provisionalLocalC() {
  LocalLinearPlant plant{};
  plant.name = "swing-native-04-C-current-vq";
  plant.a = {{{0.0, 1.0, 0.0},
              {155.128, 8.850, -2.329},
              {-317.952, 22.223, -9.244}}};
  plant.b = {{0.0, 31.421, 223.283}};
  return plant;
}

inline LocalLinearPlant provisionalLocalNominal() {
  const LocalLinearPlant b = provisionalLocalB();
  const LocalLinearPlant c = provisionalLocalC();
  LocalLinearPlant plant{};
  plant.name = "swing-native-04-BC-midpoint-current-vq";
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      plant.a[row][col] = 0.5 * (b.a[row][col] + c.a[row][col]);
    }
    plant.b[row] = 0.5 * (b.b[row] + c.b[row]);
  }
  return plant;
}

inline LocalLinearDerivative derivativeVq(const LocalLinearPlant& plant,
                                          const LocalLinearState& state,
                                          const double vq_v) {
  LocalLinearDerivative output{};
  if (!validLocalLinearPlant(plant) || !finiteState(state) ||
      !std::isfinite(vq_v)) {
    return output;
  }

  const std::array<double, 3> x{{state.theta_error_rad,
                                 state.theta_rate_rad_s,
                                 state.wheel_rate_rad_s}};
  std::array<double, 3> dx{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      dx[row] += plant.a[row][col] * x[col];
    }
    dx[row] += plant.b[row] * vq_v;
  }

  output.valid = std::isfinite(dx[0]) && std::isfinite(dx[1]) &&
                 std::isfinite(dx[2]);
  if (!output.valid) return LocalLinearDerivative{};
  output.theta_error_rate_rad_s = dx[0];
  output.theta_accel_rad_s2 = dx[1];
  output.wheel_accel_rad_s2 = dx[2];
  return output;
}

inline WheelAccelerationCoordinateResult derivativeWheelAcceleration(
    const LocalLinearPlant& plant, const LocalLinearState& state,
    const double wheel_accel_command_rad_s2) {
  WheelAccelerationCoordinateResult output{};
  if (!validLocalLinearPlant(plant) || !finiteState(state) ||
      !std::isfinite(wheel_accel_command_rad_s2)) {
    return output;
  }

  const std::array<double, 3> x{{state.theta_error_rad,
                                 state.theta_rate_rad_s,
                                 state.wheel_rate_rad_s}};
  double passive_wheel_accel_rad_s2 = 0.0;
  for (std::size_t col = 0; col < 3; ++col) {
    passive_wheel_accel_rad_s2 += plant.a[2][col] * x[col];
  }
  if (!std::isfinite(passive_wheel_accel_rad_s2)) return output;

  output.equivalent_vq_v =
      (wheel_accel_command_rad_s2 - passive_wheel_accel_rad_s2) / plant.b[2];
  if (!std::isfinite(output.equivalent_vq_v)) return output;

  std::array<double, 2> body_dx{};
  for (std::size_t row = 0; row < 2; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      body_dx[row] += plant.a[row][col] * x[col];
    }
    body_dx[row] += plant.b[row] * output.equivalent_vq_v;
  }

  if (!std::isfinite(body_dx[0]) || !std::isfinite(body_dx[1])) {
    return WheelAccelerationCoordinateResult{};
  }

  output.derivative.valid = true;
  output.derivative.theta_error_rate_rad_s = body_dx[0];
  output.derivative.theta_accel_rad_s2 = body_dx[1];
  output.derivative.wheel_accel_rad_s2 = wheel_accel_command_rad_s2;
  output.valid = true;
  return output;
}

}  // namespace triwhirl::sitl
