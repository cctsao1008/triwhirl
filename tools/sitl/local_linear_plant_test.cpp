#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string>

#include "local_linear_plant.hpp"
#include "velocity_servo_model.hpp"

namespace {

bool near(const double lhs, const double rhs,
          const double tolerance = 1.0e-10) {
  const double scale = 1.0 + std::max(std::fabs(lhs), std::fabs(rhs));
  return std::fabs(lhs - rhs) <= tolerance * scale;
}

bool expect(const bool condition, const std::string& name, int& failures) {
  if (condition) {
    std::cout << "PASS " << name << '\n';
    return true;
  }
  std::cerr << "FAIL " << name << '\n';
  ++failures;
  return false;
}

bool derivativesNear(const triwhirl::sitl::LocalLinearDerivative& lhs,
                     const triwhirl::sitl::LocalLinearDerivative& rhs) {
  return lhs.valid && rhs.valid &&
         near(lhs.theta_error_rate_rad_s, rhs.theta_error_rate_rad_s) &&
         near(lhs.theta_accel_rad_s2, rhs.theta_accel_rad_s2) &&
         near(lhs.wheel_accel_rad_s2, rhs.wheel_accel_rad_s2);
}

triwhirl::sitl::VelocityServoModel simulationServoFixture() {
  // Simulation fixture only. These values are not hardware authority, a
  // SimpleFOC tune, or a vendor-derived motor model.
  triwhirl::sitl::VelocityServoConfig config{};
  config.target_velocity_limit_rad_s = 10.0;
  config.time_constant_s = 0.1;
  config.acceleration_limit_rad_s2 = 50.0;
  return triwhirl::sitl::VelocityServoModel(config);
}

}  // namespace

int main() {
  int failures = 0;

  const std::array<triwhirl::sitl::LocalLinearPlant, 3> plants{{
      triwhirl::sitl::provisionalLocalB(),
      triwhirl::sitl::provisionalLocalC(),
      triwhirl::sitl::provisionalLocalNominal(),
  }};

  for (const auto& plant : plants) {
    expect(triwhirl::sitl::validLocalLinearPlant(plant),
           std::string(plant.name) + " fixture is transformable", failures);
  }

  const std::array<double, 3> theta_values{{-0.05, 0.0, 0.05}};
  const std::array<double, 3> rate_values{{-1.0, 0.0, 1.0}};
  const std::array<double, 3> wheel_values{{-20.0, 0.0, 20.0}};
  const std::array<double, 5> vq_values{{-3.0, -1.0, 0.0, 1.0, 3.0}};

  std::size_t parity_cases = 0U;
  bool parity_ok = true;
  for (const auto& plant : plants) {
    for (const double theta : theta_values) {
      for (const double rate : rate_values) {
        for (const double wheel : wheel_values) {
          const triwhirl::sitl::LocalLinearState state{theta, rate, wheel};
          for (const double vq : vq_values) {
            const auto original =
                triwhirl::sitl::derivativeVq(plant, state, vq);
            if (!original.valid) {
              parity_ok = false;
              continue;
            }
            const auto transformed = triwhirl::sitl::derivativeWheelAcceleration(
                plant, state, original.wheel_accel_rad_s2);
            parity_ok = parity_ok && transformed.valid &&
                        near(transformed.equivalent_vq_v, vq) &&
                        derivativesNear(original, transformed.derivative);
            ++parity_cases;
          }
        }
      }
    }
  }
  expect(parity_ok && parity_cases == 405U,
         "Vq and wheel-acceleration coordinates preserve 405 local derivatives",
         failures);

  {
    triwhirl::sitl::LocalLinearPlant invalid{};
    expect(!triwhirl::sitl::validLocalLinearPlant(invalid),
           "default local plant is invalid", failures);

    invalid = triwhirl::sitl::provisionalLocalNominal();
    invalid.b[2] = 0.0;
    const triwhirl::sitl::LocalLinearState state{};
    expect(!triwhirl::sitl::validLocalLinearPlant(invalid) &&
               !triwhirl::sitl::derivativeWheelAcceleration(invalid, state, 0.0)
                    .valid,
           "zero wheel Vq gain blocks coordinate inversion", failures);
  }

  {
    const auto nominal = triwhirl::sitl::provisionalLocalNominal();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    expect(!triwhirl::sitl::derivativeVq(
                nominal, triwhirl::sitl::LocalLinearState{nan, 0.0, 0.0}, 0.0)
                .valid &&
               !triwhirl::sitl::derivativeVq(
                nominal, triwhirl::sitl::LocalLinearState{}, inf)
                .valid &&
               !triwhirl::sitl::derivativeWheelAcceleration(
                nominal, triwhirl::sitl::LocalLinearState{0.0, -inf, 0.0}, 0.0)
                .valid &&
               !triwhirl::sitl::derivativeWheelAcceleration(
                nominal, triwhirl::sitl::LocalLinearState{}, nan)
                .valid,
           "non-finite local inputs fail closed", failures);
  }

  {
    auto invalid = triwhirl::sitl::provisionalLocalB();
    invalid.a[1][1] = std::numeric_limits<double>::infinity();
    expect(!triwhirl::sitl::validLocalLinearPlant(invalid),
           "non-finite local matrix is rejected", failures);
  }

  {
    const auto servo = simulationServoFixture();
    const triwhirl::sitl::LocalLinearState state{0.02, -0.25, 0.5};
    const auto servo_output = servo.evaluate(2.0, state.wheel_rate_rad_s);
    const auto transformed = triwhirl::sitl::derivativeWheelAcceleration(
        triwhirl::sitl::provisionalLocalNominal(), state,
        servo_output.wheel_accel_command_rad_s2);
    expect(servo_output.valid && !servo_output.target_saturated &&
               !servo_output.acceleration_saturated &&
               near(servo_output.wheel_accel_command_rad_s2, 15.0) &&
               transformed.valid && transformed.derivative.valid &&
               near(transformed.derivative.wheel_accel_rad_s2, 15.0) &&
               std::isfinite(transformed.derivative.theta_accel_rad_s2),
           "target velocity -> servo -> transformed local plant is finite",
           failures);
  }

  if (failures == 0) {
    std::cout << "PASS local linear actuator-coordinate contract" << '\n';
    return 0;
  }
  std::cerr << "FAIL local linear actuator-coordinate contract failures="
            << failures << '\n';
  return 1;
}
