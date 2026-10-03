#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "velocity_servo_model.hpp"

namespace {

constexpr double kTolerance = 1.0e-12;

bool near(const double lhs, const double rhs) {
  return std::fabs(lhs - rhs) <= kTolerance;
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

triwhirl::sitl::VelocityServoConfig simulationFixture() {
  // Simulation-only fixture. These values are deliberately round and are not
  // TriWhirl hardware authority, SimpleFOC tuning, or vendor-derived data.
  triwhirl::sitl::VelocityServoConfig config{};
  config.target_velocity_limit_rad_s = 10.0;
  config.time_constant_s = 0.1;
  config.acceleration_limit_rad_s2 = 50.0;
  return config;
}

}  // namespace

int main() {
  int failures = 0;

  {
    const triwhirl::sitl::VelocityServoModel model({});
    expect(!model.valid(), "default configuration is invalid", failures);
    expect(!model.evaluate(0.0, 0.0).valid,
           "invalid configuration cannot emit acceleration", failures);
  }

  const triwhirl::sitl::VelocityServoModel model(simulationFixture());
  expect(model.valid(), "simulation fixture is valid", failures);

  {
    const auto output = model.evaluate(3.0, 3.0);
    expect(output.valid && near(output.velocity_error_rad_s, 0.0) &&
               near(output.wheel_accel_command_rad_s2, 0.0) &&
               !output.target_saturated && !output.acceleration_saturated,
           "zero velocity error produces zero acceleration", failures);
  }

  {
    const auto positive = model.evaluate(2.0, 0.5);
    const auto negative = model.evaluate(-2.0, -0.5);
    expect(positive.valid && negative.valid &&
               near(positive.wheel_accel_command_rad_s2,
                    -negative.wheel_accel_command_rad_s2) &&
               near(positive.velocity_error_rad_s,
                    -negative.velocity_error_rad_s),
           "servo response is sign symmetric", failures);
  }

  {
    const auto output = model.evaluate(20.0, 9.0);
    expect(output.valid && output.target_saturated &&
               near(output.requested_target_velocity_rad_s, 20.0) &&
               near(output.applied_target_velocity_rad_s, 10.0) &&
               near(output.velocity_error_rad_s, 1.0) &&
               near(output.wheel_accel_command_rad_s2, 10.0) &&
               !output.acceleration_saturated,
           "target velocity is clamped before servo error", failures);
  }

  {
    const auto output = model.evaluate(8.0, 0.0);
    expect(output.valid && !output.target_saturated &&
               output.acceleration_saturated &&
               near(output.wheel_accel_command_rad_s2, 50.0),
           "wheel acceleration is bounded", failures);
  }

  {
    const auto small = model.evaluate(0.5, 0.0);
    const auto medium = model.evaluate(1.0, 0.0);
    const auto large = model.evaluate(2.0, 0.0);
    expect(small.valid && medium.valid && large.valid &&
               near(small.wheel_accel_command_rad_s2, 5.0) &&
               near(medium.wheel_accel_command_rad_s2, 10.0) &&
               near(large.wheel_accel_command_rad_s2, 20.0) &&
               small.wheel_accel_command_rad_s2 <
                   medium.wheel_accel_command_rad_s2 &&
               medium.wheel_accel_command_rad_s2 <
                   large.wheel_accel_command_rad_s2,
           "unsaturated response is monotonic in velocity error", failures);
  }

  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    expect(!model.evaluate(nan, 0.0).valid &&
               !model.evaluate(0.0, nan).valid &&
               !model.evaluate(inf, 0.0).valid &&
               !model.evaluate(0.0, -inf).valid,
           "non-finite inputs fail closed", failures);
  }

  if (failures != 0) {
    std::cerr << failures << " velocity-servo contract check(s) failed\n";
    return 1;
  }

  std::cout << "PASS target-velocity motor-servo contract\n";
  return 0;
}
