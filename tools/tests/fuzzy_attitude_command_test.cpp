#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

#include "triwhirl/fuzzy_attitude_command.hpp"
#include "triwhirl/fuzzy_balance_seed.hpp"

namespace {

bool near(const float lhs, const float rhs, const float tolerance = 1.0e-5F) {
  return std::fabs(lhs - rhs) <= tolerance;
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

triwhirl::FuzzyAttitudeCommandConfig testConfig() {
  triwhirl::FuzzyAttitudeCommandConfig config{};
  // Unit-test fixture only. These values are not firmware or hardware tuning.
  // Keep wheel input scale intentionally different from target output limit so
  // the test detects normalized-coordinate mistakes at the physical rad/s
  // command boundary.
  config.fuzzy.theta_error_scale_rad = 1.0F;
  config.fuzzy.theta_rate_scale_rad_s = 1.0F;
  config.fuzzy.wheel_velocity_scale_rad_s = 5.0F;
  config.fuzzy.target_velocity_limit_rad_s = 10.0F;
  config.fuzzy.target_velocity_singletons =
      triwhirl::fuzzy_balance::makeQualitativeRuleSeed(
          config.fuzzy.wheel_velocity_scale_rad_s,
          config.fuzzy.target_velocity_limit_rad_s);
  config.fuzzy.rule_surface_configured = true;
  config.upright_reference_rad = 1.1F;
  config.max_motor_observation_age_us = 3000U;
  return config;
}

triwhirl::FuzzyAttitudeCommandInput validInput() {
  triwhirl::FuzzyAttitudeCommandInput input{};
  input.theta_rad = 1.1F;
  input.theta_rate_rad_s = 0.0F;
  input.attitude_valid = true;
  input.now_us32 = 10000U;
  input.motor_observation_available = true;
  input.motor.shaft_velocity_rad_s = 2.5F;
  input.motor.accepted_target_velocity_rad_s = 0.0F;
  input.motor.observation_generation = 7U;
  input.motor.service_start_us32 = 9000U;
  input.motor.initialized = true;
  input.motor.sensor_valid = true;
  input.motor.backend_faulted = false;
  input.motor.actuator_enabled = false;
  input.motor.command_timed_out = false;
  return input;
}

}  // namespace

int main() {
  int failures = 0;

  {
    const triwhirl::FuzzyAttitudeCommandController controller(
        triwhirl::FuzzyAttitudeCommandConfig{});
    const auto output = controller.evaluate(validInput());
    expect(!controller.valid() && !output.target_valid &&
               output.stop_reason ==
                   triwhirl::FuzzyAttitudeCommandStopReason::kInvalidConfig,
           "default configuration fails closed", failures);

    triwhirl::MotorCommandMailbox mailbox;
    const std::uint32_t generation = controller.publish(validInput(), &mailbox);
    triwhirl::MotorCommandSnapshot command{};
    expect(generation != 0U && mailbox.tryRead(&command) && !command.enabled &&
               near(command.target_velocity_rad_s, 0.0F),
           "invalid configuration publishes explicit stop", failures);
  }

  const auto config = testConfig();
  const triwhirl::FuzzyAttitudeCommandController controller(config);
  expect(controller.valid(), "explicit test configuration is valid", failures);

  {
    const auto input = validInput();
    const auto output = controller.evaluate(input);
    expect(output.target_valid &&
               output.stop_reason ==
                   triwhirl::FuzzyAttitudeCommandStopReason::kNone &&
               near(output.theta_error_rad, 0.0F) &&
               near(output.target_velocity_rad_s, 2.5F) &&
               output.motor_observation_age_us == 1000U,
           "fresh zero-attitude state preserves wheel rad/s across unequal scales",
           failures);

    triwhirl::MotorCommandMailbox mailbox;
    const std::uint32_t generation = controller.publish(input, &mailbox);
    triwhirl::MotorCommandSnapshot command{};
    expect(generation != 0U && mailbox.tryRead(&command) && command.enabled &&
               near(command.target_velocity_rad_s,
                    output.target_velocity_rad_s) &&
               command.issued_at_us32 == input.now_us32,
           "valid fuzzy decision publishes bounded target", failures);
  }

  {
    auto a = validInput();
    a.theta_rad = config.upright_reference_rad + 0.12F;
    a.theta_rate_rad_s = 0.15F;
    a.motor.shaft_velocity_rad_s = -1.0F;

    auto b = a;
    b.theta_rad += triwhirl::kUprightPeriodRad;
    auto c = a;
    c.theta_rad -= triwhirl::kUprightPeriodRad;

    const auto out_a = controller.evaluate(a);
    const auto out_b = controller.evaluate(b);
    const auto out_c = controller.evaluate(c);
    expect(out_a.target_valid && out_b.target_valid && out_c.target_valid &&
               near(out_a.theta_error_rad, out_b.theta_error_rad) &&
               near(out_a.theta_error_rad, out_c.theta_error_rad) &&
               near(out_a.target_velocity_rad_s, out_b.target_velocity_rad_s) &&
               near(out_a.target_velocity_rad_s, out_c.target_velocity_rad_s),
           "A/B/C 120-degree uprights share one command boundary", failures);
  }

  {
    auto positive = validInput();
    positive.theta_rad = config.upright_reference_rad + 0.20F;
    positive.theta_rate_rad_s = 0.15F;
    positive.motor.shaft_velocity_rad_s = 1.5F;

    auto negative = validInput();
    negative.theta_rad = config.upright_reference_rad - 0.20F;
    negative.theta_rate_rad_s = -0.15F;
    negative.motor.shaft_velocity_rad_s = -1.5F;

    const auto pos = controller.evaluate(positive);
    const auto neg = controller.evaluate(negative);
    expect(pos.target_valid && neg.target_valid &&
               near(pos.theta_error_rad, -neg.theta_error_rad) &&
               near(pos.target_velocity_rad_s, -neg.target_velocity_rad_s),
           "mirrored mechanical state preserves odd command symmetry", failures);
  }

  {
    auto input = validInput();
    input.now_us32 = 13001U;
    const auto output = controller.evaluate(input);
    expect(!output.target_valid &&
               output.stop_reason ==
                   triwhirl::FuzzyAttitudeCommandStopReason::kMotorObservationStale &&
               output.motor_observation_age_us == 4001U,
           "stale motor observation fails closed", failures);
  }

  {
    auto input = validInput();
    input.motor.service_start_us32 = 0xFFFFFFF0U;
    input.now_us32 = 0x00000020U;
    const auto output = controller.evaluate(input);
    expect(output.target_valid && output.motor_observation_age_us == 48U,
           "motor observation freshness is correct across uint32 wrap",
           failures);
  }

  {
    auto input = validInput();
    input.attitude_valid = false;
    expect(controller.evaluate(input).stop_reason ==
               triwhirl::FuzzyAttitudeCommandStopReason::kInvalidAttitude,
           "invalid attitude flag fails closed", failures);

    input = validInput();
    input.theta_rate_rad_s = std::numeric_limits<float>::quiet_NaN();
    expect(controller.evaluate(input).stop_reason ==
               triwhirl::FuzzyAttitudeCommandStopReason::kInvalidAttitude,
           "non-finite attitude state fails closed", failures);

    input = validInput();
    input.motor_observation_available = false;
    expect(controller.evaluate(input).stop_reason ==
               triwhirl::FuzzyAttitudeCommandStopReason::kMotorObservationUnavailable,
           "missing motor observation fails closed", failures);

    input = validInput();
    input.motor.initialized = false;
    expect(controller.evaluate(input).stop_reason ==
               triwhirl::FuzzyAttitudeCommandStopReason::kMotorNotInitialized,
           "uninitialized motor domain fails closed", failures);

    input = validInput();
    input.motor.sensor_valid = false;
    expect(controller.evaluate(input).stop_reason ==
               triwhirl::FuzzyAttitudeCommandStopReason::kMotorSensorInvalid,
           "invalid motor sensor fails closed", failures);

    input = validInput();
    input.motor.backend_faulted = true;
    expect(controller.evaluate(input).stop_reason ==
               triwhirl::FuzzyAttitudeCommandStopReason::kMotorBackendFault,
           "motor backend fault fails closed", failures);

    input = validInput();
    input.motor.command_timed_out = true;
    expect(controller.evaluate(input).stop_reason ==
               triwhirl::FuzzyAttitudeCommandStopReason::kMotorCommandTimedOut,
           "motor command timeout fails closed", failures);
  }

  {
    auto overflow_config = testConfig();
    overflow_config.fuzzy.wheel_velocity_scale_rad_s =
        std::numeric_limits<float>::min();
    const triwhirl::FuzzyAttitudeCommandController overflow_controller(
        overflow_config);
    auto input = validInput();
    input.motor.shaft_velocity_rad_s = std::numeric_limits<float>::max();
    const auto output = overflow_controller.evaluate(input);
    expect(overflow_controller.valid() && !output.target_valid &&
               output.stop_reason ==
                   triwhirl::FuzzyAttitudeCommandStopReason::kInvalidFuzzyOutput,
           "normalization overflow fails closed instead of emitting a target",
           failures);
  }

  {
    auto invalid_config = testConfig();
    invalid_config.max_motor_observation_age_us = 0x80000000U;
    expect(!triwhirl::validFuzzyAttitudeCommandConfig(invalid_config),
           "ambiguous uint32 freshness interval is rejected", failures);
  }

  if (failures == 0) {
    std::cout << "PASS pure-fuzzy attitude command boundary contract\n";
    return 0;
  }
  std::cerr << "FAIL pure-fuzzy attitude command boundary failures=" << failures
            << '\n';
  return 1;
}
