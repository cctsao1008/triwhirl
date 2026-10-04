#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>

#include "triwhirl/fuzzy_attitude_command.hpp"
#include "triwhirl/fuzzy_balance_seed.hpp"
#include "triwhirl/motor_execution.hpp"

namespace {

constexpr float kReferenceRad = 1.1F;
constexpr float kThetaScaleRad = 0.50F;
constexpr float kThetaRateScaleRadS = 2.0F;
constexpr float kWheelScaleRadS = 20.0F;
constexpr float kTargetLimitRadS = 40.0F;
constexpr std::uint32_t kMaxObservationAgeUs = 1500U;
constexpr std::uint32_t kCommandTimeoutUs = 2000U;

bool near(const float lhs, const float rhs, const float tolerance = 1.0e-4F) {
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

struct FakeMechanicalMotor {
  bool initialized = false;
  bool requested_enabled = false;
  bool actuator_enabled = false;
  bool sensor_valid = true;
  bool backend_faulted = false;
  float requested_target_rad_s = 0.0F;
  float shaft_velocity_rad_s = 4.0F;
  int begin_calls = 0;
  int service_calls = 0;
  int command_calls = 0;
  int stop_calls = 0;
};

bool fakeBegin(void* context) {
  auto* motor = static_cast<FakeMechanicalMotor*>(context);
  ++motor->begin_calls;
  motor->initialized = true;
  return true;
}

void fakeService(void* context) {
  auto* motor = static_cast<FakeMechanicalMotor*>(context);
  ++motor->service_calls;
  motor->actuator_enabled = motor->requested_enabled;

  // Deterministic host-only mechanical surrogate: an enabled command is made
  // observable as the measured shaft velocity on the same service invocation.
  // This is intentionally NOT a motor-dynamics model or hardware tuning claim.
  if (motor->requested_enabled && motor->sensor_valid &&
      !motor->backend_faulted) {
    motor->shaft_velocity_rad_s = motor->requested_target_rad_s;
  }
}

bool fakeCommand(void* context, const float target_velocity_rad_s) {
  auto* motor = static_cast<FakeMechanicalMotor*>(context);
  ++motor->command_calls;
  motor->requested_target_rad_s = target_velocity_rad_s;
  motor->requested_enabled = true;
  return true;
}

void fakeStop(void* context) {
  auto* motor = static_cast<FakeMechanicalMotor*>(context);
  ++motor->stop_calls;
  motor->requested_target_rad_s = 0.0F;
  motor->requested_enabled = false;
}

triwhirl::MotorControlObservation fakeObservation(void* context) {
  const auto* motor = static_cast<FakeMechanicalMotor*>(context);
  triwhirl::MotorControlObservation observation{};
  observation.shaft_velocity_rad_s = motor->shaft_velocity_rad_s;
  observation.target_velocity_rad_s =
      motor->requested_enabled ? motor->requested_target_rad_s : 0.0F;
  observation.initialized = motor->initialized;
  observation.sensor_valid = motor->sensor_valid;
  observation.command_enabled = motor->requested_enabled;
  observation.actuator_enabled = motor->actuator_enabled;
  observation.backend_faulted = motor->backend_faulted;
  return observation;
}

triwhirl::MotorControl makeFakeMotorControl(FakeMechanicalMotor* motor) {
  triwhirl::MotorControlOps ops{};
  ops.begin = fakeBegin;
  ops.service_backend = fakeService;
  ops.command_target_velocity = fakeCommand;
  ops.stop = fakeStop;
  ops.observation = fakeObservation;
  return triwhirl::MotorControl(motor, ops, kTargetLimitRadS);
}

triwhirl::FuzzyAttitudeCommandConfig fuzzyConfig() {
  triwhirl::FuzzyAttitudeCommandConfig config{};
  config.fuzzy.theta_error_scale_rad = kThetaScaleRad;
  config.fuzzy.theta_rate_scale_rad_s = kThetaRateScaleRadS;
  config.fuzzy.wheel_velocity_scale_rad_s = kWheelScaleRadS;
  config.fuzzy.target_velocity_limit_rad_s = kTargetLimitRadS;
  config.fuzzy.target_velocity_singletons =
      triwhirl::fuzzy_balance::makeQualitativeRuleSeed(
          kWheelScaleRadS, kTargetLimitRadS);
  config.fuzzy.rule_surface_configured = true;
  config.upright_reference_rad = kReferenceRad;
  config.max_motor_observation_age_us = kMaxObservationAgeUs;
  return config;
}

struct Harness {
  FakeMechanicalMotor motor{};
  triwhirl::MotorCommandMailbox commands{};
  triwhirl::MotorObservationMailbox observations{};
  triwhirl::MotorExecutionConfig execution_config{};
  triwhirl::MotorExecutionDomain executor;
  triwhirl::FuzzyAttitudeCommandController fuzzy;

  Harness()
      : execution_config{.command_timeout_us = kCommandTimeoutUs},
        executor(makeFakeMotorControl(&motor), &commands, &observations,
                 execution_config),
        fuzzy(fuzzyConfig()) {}
};

bool readObservation(Harness* harness,
                     triwhirl::MotorObservationSnapshot* output) {
  return harness != nullptr && output != nullptr &&
         harness->observations.tryRead(output);
}

triwhirl::FuzzyAttitudeCommandInput makeInput(
    const triwhirl::MotorObservationSnapshot& motor, const float theta_rad,
    const float theta_rate_rad_s, const std::uint32_t now_us32) {
  triwhirl::FuzzyAttitudeCommandInput input{};
  input.theta_rad = theta_rad;
  input.theta_rate_rad_s = theta_rate_rad_s;
  input.attitude_valid = true;
  input.now_us32 = now_us32;
  input.motor_observation_available = true;
  input.motor = motor;
  return input;
}

float acceptedTargetAfterFreshRoundTrip(const float theta_rad, bool* valid) {
  Harness harness;
  *valid = false;
  if (!harness.executor.begin(1000U)) return 0.0F;

  triwhirl::MotorObservationSnapshot observation{};
  if (!readObservation(&harness, &observation)) return 0.0F;

  const auto input = makeInput(observation, theta_rad, 0.10F, 1300U);
  const auto decision = harness.fuzzy.evaluate(input);
  if (!decision.target_valid) return 0.0F;
  const std::uint32_t generation = harness.fuzzy.publish(input, &harness.commands);
  if (generation == 0U) return 0.0F;

  harness.executor.service(1400U);
  if (!readObservation(&harness, &observation)) return 0.0F;
  if (observation.consumed_command_generation != generation ||
      !observation.actuator_enabled) {
    return 0.0F;
  }

  *valid = true;
  return observation.accepted_target_velocity_rad_s;
}

}  // namespace

int main() {
  int failures = 0;
  std::cout << "fuzzy_motor_roundtrip_authority=HOST_ORCHESTRATION_ONLY\n"
            << "hardware_balance_authority=NONE\n";

  Harness harness;
  expect(harness.executor.valid() && harness.fuzzy.valid(),
         "round-trip harness is explicitly configured", failures);
  expect(harness.executor.begin(1000U),
         "motor executor begins successfully", failures);

  triwhirl::MotorObservationSnapshot observation{};
  expect(readObservation(&harness, &observation) && observation.initialized &&
             observation.sensor_valid && !observation.backend_faulted &&
             !observation.actuator_enabled && !observation.command_timed_out,
         "executor begins de-energized with a valid mechanical observation",
         failures);

  const auto first_input =
      makeInput(observation, kReferenceRad + 0.05F, 0.0F, 1400U);
  const auto first_decision = harness.fuzzy.evaluate(first_input);
  expect(first_decision.target_valid &&
             std::fabs(first_decision.target_velocity_rad_s) <=
                 kTargetLimitRadS,
         "fresh observation produces only a bounded velocity target", failures);

  const std::uint32_t first_generation =
      harness.fuzzy.publish(first_input, &harness.commands);
  triwhirl::MotorCommandSnapshot first_command{};
  expect(first_generation != 0U && harness.commands.tryRead(&first_command) &&
             first_command.enabled &&
             near(first_command.target_velocity_rad_s,
                  first_decision.target_velocity_rad_s),
         "fuzzy publisher writes the evaluated target into the command mailbox",
         failures);

  harness.executor.service(1600U);
  expect(readObservation(&harness, &observation) &&
             observation.consumed_command_generation == first_generation &&
             near(observation.accepted_target_velocity_rad_s,
                  first_decision.target_velocity_rad_s) &&
             observation.actuator_enabled && !observation.command_timed_out,
         "motor executor consumes the exact target and reports its generation",
         failures);

  const auto second_input =
      makeInput(observation, kReferenceRad, 0.0F, 1800U);
  const auto second_decision = harness.fuzzy.evaluate(second_input);
  expect(second_decision.target_valid &&
             near(second_decision.target_velocity_rad_s,
                  observation.shaft_velocity_rad_s),
         "next fuzzy tick consumes the returned motor observation in physical rad/s",
         failures);

  const std::uint32_t second_generation =
      harness.fuzzy.publish(second_input, &harness.commands);
  harness.executor.service(1900U);
  expect(readObservation(&harness, &observation) &&
             observation.consumed_command_generation == second_generation &&
             observation.actuator_enabled,
         "second command completes the mailbox-executor-observation round trip",
         failures);

  const auto stale_input = makeInput(
      observation, kReferenceRad, 0.0F,
      observation.service_start_us32 + kMaxObservationAgeUs + 1U);
  const auto stale_decision = harness.fuzzy.evaluate(stale_input);
  const std::uint32_t stale_stop_generation =
      harness.fuzzy.publish(stale_input, &harness.commands);
  expect(!stale_decision.target_valid &&
             stale_decision.stop_reason ==
                 triwhirl::FuzzyAttitudeCommandStopReason::kMotorObservationStale &&
             stale_stop_generation != 0U,
         "stale observation becomes an explicit stop command", failures);

  const std::uint32_t stale_stop_service_us = stale_input.now_us32 + 100U;
  harness.executor.service(stale_stop_service_us);
  expect(readObservation(&harness, &observation) &&
             observation.consumed_command_generation == stale_stop_generation &&
             !observation.actuator_enabled && !observation.command_timed_out,
         "executor consumes stale-observation stop and returns fresh disabled state",
         failures);

  const auto stale_recovery_input = makeInput(
      observation, kReferenceRad, 0.0F, stale_stop_service_us + 100U);
  expect(harness.fuzzy.evaluate(stale_recovery_input).target_valid,
         "fresh post-stop observation removes stale-state bootstrap deadlock",
         failures);

  const std::uint32_t recovery_generation =
      harness.fuzzy.publish(stale_recovery_input, &harness.commands);
  harness.executor.service(stale_stop_service_us + 200U);
  expect(readObservation(&harness, &observation) &&
             observation.consumed_command_generation == recovery_generation &&
             observation.actuator_enabled,
         "controller can re-establish target authority after explicit stale stop",
         failures);

  const std::uint32_t timeout_service_us =
      observation.service_start_us32 + kCommandTimeoutUs + 1U;
  harness.executor.service(timeout_service_us);
  expect(readObservation(&harness, &observation) &&
             observation.command_timed_out && !observation.actuator_enabled,
         "execution-domain command timeout de-energizes the fake actuator",
         failures);

  const auto timeout_input = makeInput(
      observation, kReferenceRad, 0.0F, timeout_service_us + 100U);
  const auto timeout_decision = harness.fuzzy.evaluate(timeout_input);
  const std::uint32_t timeout_stop_generation =
      harness.fuzzy.publish(timeout_input, &harness.commands);
  expect(!timeout_decision.target_valid &&
             timeout_decision.stop_reason ==
                 triwhirl::FuzzyAttitudeCommandStopReason::kMotorCommandTimedOut &&
             timeout_stop_generation != 0U,
         "timed-out motor observation causes fuzzy side to publish stop",
         failures);

  harness.executor.service(timeout_service_us + 200U);
  expect(readObservation(&harness, &observation) &&
             observation.consumed_command_generation == timeout_stop_generation &&
             !observation.command_timed_out && !observation.actuator_enabled,
         "consuming explicit stop clears timeout state without energizing",
         failures);

  const auto timeout_recovery_input = makeInput(
      observation, kReferenceRad, 0.0F, timeout_service_us + 300U);
  expect(harness.fuzzy.evaluate(timeout_recovery_input).target_valid,
         "post-timeout stop yields a valid recovery observation", failures);

  const std::uint32_t post_timeout_generation =
      harness.fuzzy.publish(timeout_recovery_input, &harness.commands);
  harness.executor.service(timeout_service_us + 400U);
  expect(readObservation(&harness, &observation) &&
             observation.consumed_command_generation == post_timeout_generation &&
             observation.actuator_enabled,
         "timeout recovery can explicitly restore target authority", failures);

  harness.motor.sensor_valid = false;
  harness.executor.service(timeout_service_us + 500U);
  expect(readObservation(&harness, &observation) &&
             !observation.sensor_valid && !observation.actuator_enabled &&
             !observation.backend_faulted,
         "invalid motor sensor forces same-invocation de-energizing", failures);

  const auto invalid_sensor_input = makeInput(
      observation, kReferenceRad, 0.0F, timeout_service_us + 600U);
  const auto invalid_sensor_decision = harness.fuzzy.evaluate(invalid_sensor_input);
  const std::uint32_t invalid_sensor_stop_generation =
      harness.fuzzy.publish(invalid_sensor_input, &harness.commands);
  expect(!invalid_sensor_decision.target_valid &&
             invalid_sensor_decision.stop_reason ==
                 triwhirl::FuzzyAttitudeCommandStopReason::kMotorSensorInvalid &&
             invalid_sensor_stop_generation != 0U,
         "invalid motor sensor propagates through fuzzy boundary as stop",
         failures);

  harness.motor.sensor_valid = true;
  harness.executor.service(timeout_service_us + 700U);
  expect(readObservation(&harness, &observation) && observation.sensor_valid &&
             !observation.actuator_enabled &&
             observation.consumed_command_generation ==
                 invalid_sensor_stop_generation,
         "sensor recovery plus explicit stop returns a clean mechanical observation",
         failures);

  const auto sensor_recovery_input = makeInput(
      observation, kReferenceRad, 0.0F, timeout_service_us + 800U);
  expect(harness.fuzzy.evaluate(sensor_recovery_input).target_valid,
         "sensor recovery does not require hidden controller state reset", failures);

  harness.fuzzy.publish(sensor_recovery_input, &harness.commands);
  harness.executor.service(timeout_service_us + 900U);
  harness.motor.backend_faulted = true;
  harness.executor.service(timeout_service_us + 1000U);
  expect(readObservation(&harness, &observation) &&
             observation.backend_faulted && !observation.actuator_enabled,
         "backend fault latches and de-energizes before returning observation",
         failures);

  const auto backend_fault_input = makeInput(
      observation, kReferenceRad, 0.0F, timeout_service_us + 1100U);
  const auto backend_fault_decision = harness.fuzzy.evaluate(backend_fault_input);
  expect(!backend_fault_decision.target_valid &&
             backend_fault_decision.stop_reason ==
                 triwhirl::FuzzyAttitudeCommandStopReason::kMotorBackendFault,
         "latched backend fault propagates as a fail-closed fuzzy decision",
         failures);

  bool equivalent_a_valid = false;
  bool equivalent_b_valid = false;
  constexpr float kEquivalentDeltaRad = 0.08F;
  const float target_a = acceptedTargetAfterFreshRoundTrip(
      kReferenceRad + kEquivalentDeltaRad, &equivalent_a_valid);
  const float target_b = acceptedTargetAfterFreshRoundTrip(
      kReferenceRad + triwhirl::kUprightPeriodRad + kEquivalentDeltaRad,
      &equivalent_b_valid);
  expect(equivalent_a_valid && equivalent_b_valid && near(target_a, target_b),
         "120-degree-equivalent attitudes remain equivalent through full round trip",
         failures);

  if (failures == 0) {
    std::cout << "PASS fuzzy attitude -> mailbox -> motor executor round-trip contract\n";
    return 0;
  }

  std::cerr << "FAIL fuzzy motor round-trip contract failures=" << failures
            << '\n';
  return 1;
}
