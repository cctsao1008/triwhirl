#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

#include "local_linear_plant.hpp"
#include "triwhirl/fuzzy_balance_controller.hpp"
#include "triwhirl/fuzzy_balance_seed.hpp"
#include "velocity_servo_model.hpp"

namespace {

constexpr double kStepS = 0.0005;
constexpr double kDurationS = 5.0;
constexpr int kSteps = static_cast<int>(kDurationS / kStepS);
constexpr double kMirrorTolerance = 5.0e-5;

bool expect(const bool condition, const std::string& name, int& failures) {
  if (condition) {
    std::cout << "PASS " << name << '\n';
    return true;
  }
  std::cerr << "FAIL " << name << '\n';
  ++failures;
  return false;
}

bool near(const double lhs, const double rhs,
          const double tolerance = kMirrorTolerance) {
  return std::fabs(lhs - rhs) <= tolerance;
}

triwhirl::FuzzyBalanceConfig fuzzySimulationFixture() {
  triwhirl::FuzzyBalanceConfig config{};

  // Simulation-only normalization fixture selected to exercise the complete
  // pure-fuzzy target-velocity path on the historical nominal local model.
  // These values are NOT firmware tuning, hardware authority, or SimpleFOC
  // commissioning data.
  config.theta_error_scale_rad = 0.12F;
  config.theta_rate_scale_rad_s = 1.30F;
  config.wheel_velocity_scale_rad_s = 17.0F;
  config.target_velocity_limit_rad_s = 80.0F;
  config.target_velocity_singletons =
      triwhirl::fuzzy_balance::makeQualitativeRuleSeed();
  config.rule_surface_configured = true;
  return config;
}

triwhirl::sitl::VelocityServoConfig servoSimulationFixture() {
  triwhirl::sitl::VelocityServoConfig config{};

  // Mechanical-domain simulation surrogate only. These are not SimpleFOC
  // PI/LPF settings and must not be copied into the real motor path.
  config.target_velocity_limit_rad_s = 80.0;
  config.time_constant_s = 0.10;
  config.acceleration_limit_rad_s2 = 150.0;
  return config;
}

struct ScenarioResult {
  bool valid = false;
  triwhirl::sitl::LocalLinearState final_state{};
  double max_abs_theta_rad = 0.0;
  double max_abs_wheel_rate_rad_s = 0.0;
  double max_abs_target_velocity_rad_s = 0.0;
  bool target_saturated = false;
  bool acceleration_saturated = false;
};

ScenarioResult runScenario(const triwhirl::sitl::LocalLinearState& initial) {
  ScenarioResult result{};
  triwhirl::sitl::LocalLinearState state = initial;

  const triwhirl::FuzzyBalanceController fuzzy(fuzzySimulationFixture());
  const triwhirl::sitl::VelocityServoModel servo(servoSimulationFixture());
  const triwhirl::sitl::LocalLinearPlant plant =
      triwhirl::sitl::provisionalLocalNominal();

  if (!fuzzy.valid() || !servo.valid() ||
      !triwhirl::sitl::validLocalLinearPlant(plant)) {
    return result;
  }

  result.max_abs_theta_rad = std::fabs(state.theta_error_rad);
  result.max_abs_wheel_rate_rad_s = std::fabs(state.wheel_rate_rad_s);

  for (int step = 0; step < kSteps; ++step) {
    const auto fuzzy_output = fuzzy.evaluate(triwhirl::FuzzyBalanceInput{
        static_cast<float>(state.theta_error_rad),
        static_cast<float>(state.theta_rate_rad_s),
        static_cast<float>(state.wheel_rate_rad_s)});
    if (!fuzzy_output.valid) return ScenarioResult{};

    const auto servo_output = servo.evaluate(
        static_cast<double>(fuzzy_output.target_velocity_rad_s),
        state.wheel_rate_rad_s);
    if (!servo_output.valid) return ScenarioResult{};

    const auto plant_output = triwhirl::sitl::derivativeWheelAcceleration(
        plant, state, servo_output.wheel_accel_command_rad_s2);
    if (!plant_output.valid || !plant_output.derivative.valid) {
      return ScenarioResult{};
    }

    state.theta_error_rad +=
        kStepS * plant_output.derivative.theta_error_rate_rad_s;
    state.theta_rate_rad_s +=
        kStepS * plant_output.derivative.theta_accel_rad_s2;
    state.wheel_rate_rad_s +=
        kStepS * plant_output.derivative.wheel_accel_rad_s2;

    if (!triwhirl::sitl::finiteState(state)) return ScenarioResult{};

    result.max_abs_theta_rad =
        std::max(result.max_abs_theta_rad, std::fabs(state.theta_error_rad));
    result.max_abs_wheel_rate_rad_s = std::max(
        result.max_abs_wheel_rate_rad_s, std::fabs(state.wheel_rate_rad_s));
    result.max_abs_target_velocity_rad_s =
        std::max(result.max_abs_target_velocity_rad_s,
                 std::fabs(static_cast<double>(
                     fuzzy_output.target_velocity_rad_s)));
    result.target_saturated =
        result.target_saturated || servo_output.target_saturated;
    result.acceleration_saturated =
        result.acceleration_saturated || servo_output.acceleration_saturated;
  }

  result.final_state = state;
  result.valid = true;
  return result;
}

bool recovered(const ScenarioResult& result) {
  return result.valid && result.max_abs_theta_rad < 0.09 &&
         std::fabs(result.final_state.theta_error_rad) < 1.0e-3 &&
         std::fabs(result.final_state.theta_rate_rad_s) < 2.0e-3 &&
         result.max_abs_target_velocity_rad_s <= 80.0 + 1.0e-6;
}

}  // namespace

int main() {
  int failures = 0;

  std::cout
      << "pure_fuzzy_sitl_authority=SIMULATION_ONLY_HISTORICAL_NOMINAL\n"
      << "hardware_tune_authority=NONE\n"
      << "controller_path=fuzzy->target_velocity->velocity_servo->local_plant\n";

  const ScenarioResult positive = runScenario({0.02, 0.0, 0.0});
  const ScenarioResult negative = runScenario({-0.02, 0.0, 0.0});
  const ScenarioResult rate_kick = runScenario({0.0, 0.10, 0.0});
  const ScenarioResult wheel_kick = runScenario({0.0, 0.0, 1.0});

  expect(recovered(positive),
         "positive small-angle disturbance recovers on simulation fixture",
         failures);
  expect(recovered(negative),
         "negative small-angle disturbance recovers on simulation fixture",
         failures);
  expect(recovered(rate_kick),
         "body-rate disturbance recovers on simulation fixture", failures);
  expect(recovered(wheel_kick),
         "wheel-rate disturbance recovers on simulation fixture", failures);

  expect(positive.valid && negative.valid &&
             near(positive.final_state.theta_error_rad,
                  -negative.final_state.theta_error_rad) &&
             near(positive.final_state.theta_rate_rad_s,
                  -negative.final_state.theta_rate_rad_s) &&
             near(positive.final_state.wheel_rate_rad_s,
                  -negative.final_state.wheel_rate_rad_s) &&
             near(positive.max_abs_theta_rad, negative.max_abs_theta_rad) &&
             near(positive.max_abs_wheel_rate_rad_s,
                  negative.max_abs_wheel_rate_rad_s),
         "mirrored disturbances produce mirrored closed-loop response",
         failures);

  expect(positive.valid && !positive.target_saturated &&
             !positive.acceleration_saturated,
         "reference small-angle case remains inside simulation servo limits",
         failures);

  if (failures == 0) {
    std::cout << "PASS pure-fuzzy near-upright target-velocity SITL\n";
    return 0;
  }

  std::cerr << "FAIL pure-fuzzy near-upright SITL failures=" << failures
            << '\n';
  return 1;
}
