#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "triwhirl/motor_execution.hpp"

namespace {

struct FakeBackend {
  bool begin_result = true;
  bool command_result = true;
  bool initialized = false;
  bool requested_enabled = false;
  bool actuator_enabled = false;
  bool sensor_valid = true;
  bool backend_faulted = false;
  bool force_stuck_enabled = false;
  float requested_target_rad_s = 0.0F;
  float shaft_velocity_rad_s = 3.0F;
  int begin_calls = 0;
  int command_calls = 0;
  int stop_calls = 0;
  int service_calls = 0;
};

bool begin(void* context) {
  auto* backend = static_cast<FakeBackend*>(context);
  ++backend->begin_calls;
  backend->initialized = backend->begin_result;
  return backend->begin_result;
}

void service(void* context) {
  auto* backend = static_cast<FakeBackend*>(context);
  ++backend->service_calls;
  backend->actuator_enabled =
      backend->force_stuck_enabled ? true : backend->requested_enabled;
}

bool command(void* context, const float target_rad_s) {
  auto* backend = static_cast<FakeBackend*>(context);
  ++backend->command_calls;
  if (!backend->command_result) {
    return false;
  }
  backend->requested_target_rad_s = target_rad_s;
  backend->requested_enabled = true;
  return true;
}

void stop(void* context) {
  auto* backend = static_cast<FakeBackend*>(context);
  ++backend->stop_calls;
  backend->requested_target_rad_s = 0.0F;
  backend->requested_enabled = false;
  // Deliberately do not change actuator_enabled here. The fake mirrors the
  // SimpleFOC probe contract: stop is a request, service enacts de-energizing.
}

triwhirl::MotorControlObservation observation(void* context) {
  const auto* backend = static_cast<FakeBackend*>(context);
  triwhirl::MotorControlObservation result{};
  result.shaft_velocity_rad_s = backend->shaft_velocity_rad_s;
  result.target_velocity_rad_s =
      backend->requested_enabled ? backend->requested_target_rad_s : 0.0F;
  result.initialized = backend->initialized;
  result.sensor_valid = backend->sensor_valid;
  result.command_enabled = backend->requested_enabled;
  result.actuator_enabled = backend->actuator_enabled;
  result.backend_faulted = backend->backend_faulted;
  return result;
}

triwhirl::MotorControl makeControl(FakeBackend* backend,
                                   const float limit_rad_s = 80.0F) {
  triwhirl::MotorControlOps ops{};
  ops.begin = begin;
  ops.service_backend = service;
  ops.command_target_velocity = command;
  ops.stop = stop;
  ops.observation = observation;
  return triwhirl::MotorControl(backend, ops, limit_rad_s);
}

triwhirl::MotorObservationSnapshot readObservation(
    triwhirl::MotorObservationMailbox* mailbox) {
  triwhirl::MotorObservationSnapshot result{};
  assert(mailbox->tryRead(&result));
  return result;
}

void testBeginStartsDeenergized() {
  FakeBackend backend{};
  triwhirl::MotorCommandMailbox commands;
  triwhirl::MotorObservationMailbox observations;
  triwhirl::MotorExecutionDomain executor(makeControl(&backend), &commands,
                                           &observations);

  assert(executor.valid());
  assert(executor.begin(100U));
  assert(executor.started());
  assert(backend.begin_calls == 1);
  assert(backend.stop_calls == 1);
  assert(backend.service_calls == 1);
  assert(!backend.actuator_enabled);

  const auto snapshot = readObservation(&observations);
  assert(snapshot.initialized);
  assert(snapshot.sensor_valid);
  assert(!snapshot.backend_faulted);
  assert(!snapshot.actuator_enabled);
  assert(snapshot.applied_command_generation == 0U);
}

void testTargetAndStopAreAppliedOnMotorService() {
  FakeBackend backend{};
  triwhirl::MotorCommandMailbox commands;
  triwhirl::MotorObservationMailbox observations;
  triwhirl::MotorExecutionDomain executor(makeControl(&backend), &commands,
                                           &observations);
  assert(executor.begin(0U));

  const uint32_t target_generation = commands.publishTarget(20.0F, 1000U);
  assert(target_generation == 1U);
  executor.service(1250U);
  auto snapshot = readObservation(&observations);
  assert(backend.command_calls == 1);
  assert(backend.actuator_enabled);
  assert(snapshot.actuator_enabled);
  assert(snapshot.applied_command_generation == target_generation);
  assert(snapshot.command_apply_latency_us == 250U);
  assert(snapshot.applied_target_velocity_rad_s == 20.0F);
  assert(!snapshot.command_timed_out);

  const uint32_t stop_generation = commands.publishStop(1500U);
  assert(stop_generation == 2U);
  const int services_before_stop = backend.service_calls;
  executor.service(1600U);
  snapshot = readObservation(&observations);
  assert(backend.service_calls == services_before_stop + 1);
  assert(!backend.actuator_enabled);
  assert(!snapshot.actuator_enabled);
  assert(snapshot.applied_command_generation == stop_generation);
  assert(snapshot.command_apply_latency_us == 100U);
  assert(snapshot.applied_target_velocity_rad_s == 0.0F);
  assert(executor.stats().applied_commands == 1U);
  assert(executor.stats().applied_stops == 1U);
}

void testLatestCommandWinsWithoutQueueBacklog() {
  FakeBackend backend{};
  triwhirl::MotorCommandMailbox commands;
  triwhirl::MotorObservationMailbox observations;
  triwhirl::MotorExecutionDomain executor(makeControl(&backend), &commands,
                                           &observations);
  assert(executor.begin(0U));

  assert(commands.publishTarget(10.0F, 100U) == 1U);
  assert(commands.publishTarget(30.0F, 200U) == 2U);
  executor.service(260U);

  const auto snapshot = readObservation(&observations);
  assert(backend.command_calls == 1);
  assert(backend.requested_target_rad_s == 30.0F);
  assert(snapshot.applied_command_generation == 2U);
  assert(snapshot.command_apply_latency_us == 60U);
}

void testCommandTimeoutStopsInOneServiceInvocation() {
  FakeBackend backend{};
  triwhirl::MotorCommandMailbox commands;
  triwhirl::MotorObservationMailbox observations;
  triwhirl::MotorExecutionConfig config{};
  config.command_timeout_us = 1000U;
  triwhirl::MotorExecutionDomain executor(makeControl(&backend), &commands,
                                           &observations, config);
  assert(executor.begin(0U));

  assert(commands.publishTarget(12.0F, 100U) == 1U);
  executor.service(500U);
  assert(readObservation(&observations).actuator_enabled);

  const int services_before_timeout = backend.service_calls;
  executor.service(1201U);
  const auto snapshot = readObservation(&observations);
  assert(backend.service_calls == services_before_timeout + 1);
  assert(!snapshot.actuator_enabled);
  assert(snapshot.command_timed_out);
  assert(snapshot.applied_target_velocity_rad_s == 0.0F);
  assert(executor.stats().timeout_stops == 1U);
}

void testAlreadyExpiredNewTargetNeverEnergizes() {
  FakeBackend backend{};
  triwhirl::MotorCommandMailbox commands;
  triwhirl::MotorObservationMailbox observations;
  triwhirl::MotorExecutionConfig config{};
  config.command_timeout_us = 1000U;
  triwhirl::MotorExecutionDomain executor(makeControl(&backend), &commands,
                                           &observations, config);
  assert(executor.begin(0U));

  assert(commands.publishTarget(25.0F, 100U) == 1U);
  executor.service(1201U);
  const auto snapshot = readObservation(&observations);
  assert(backend.command_calls == 0);
  assert(!snapshot.actuator_enabled);
  assert(snapshot.command_timed_out);
  assert(snapshot.applied_command_generation == 1U);
  assert(snapshot.command_apply_latency_us == 1101U);
}

void testSensorInvalidityForcesSameInvocationStop() {
  FakeBackend backend{};
  triwhirl::MotorCommandMailbox commands;
  triwhirl::MotorObservationMailbox observations;
  triwhirl::MotorExecutionDomain executor(makeControl(&backend), &commands,
                                           &observations);
  assert(executor.begin(0U));
  assert(commands.publishTarget(15.0F, 100U) == 1U);
  executor.service(150U);
  assert(backend.actuator_enabled);

  backend.sensor_valid = false;
  const int services_before_fault = backend.service_calls;
  executor.service(200U);
  const auto snapshot = readObservation(&observations);
  // One normal service observes invalid sensor while energized, then one
  // bounded fail-safe service enacts stop in the same executor invocation.
  assert(backend.service_calls == services_before_fault + 2);
  assert(!snapshot.sensor_valid);
  assert(!snapshot.actuator_enabled);
  assert(!snapshot.backend_faulted);
  assert(executor.stats().safety_stops == 1U);
}

void testBackendFaultLatchesAndPreventsReenable() {
  FakeBackend backend{};
  triwhirl::MotorCommandMailbox commands;
  triwhirl::MotorObservationMailbox observations;
  triwhirl::MotorExecutionDomain executor(makeControl(&backend), &commands,
                                           &observations);
  assert(executor.begin(0U));
  assert(commands.publishTarget(10.0F, 100U) == 1U);
  executor.service(120U);
  assert(backend.actuator_enabled);

  backend.backend_faulted = true;
  executor.service(150U);
  auto snapshot = readObservation(&observations);
  assert(executor.backendFaultLatched());
  assert(snapshot.backend_faulted);
  assert(!snapshot.actuator_enabled);

  backend.backend_faulted = false;
  const int command_calls_before = backend.command_calls;
  assert(commands.publishTarget(20.0F, 200U) == 2U);
  executor.service(220U);
  snapshot = readObservation(&observations);
  assert(backend.command_calls == command_calls_before);
  assert(snapshot.backend_faulted);
  assert(!snapshot.actuator_enabled);
}

void testBackendCommandRejectionFailsClosed() {
  FakeBackend backend{};
  backend.command_result = false;
  triwhirl::MotorCommandMailbox commands;
  triwhirl::MotorObservationMailbox observations;
  triwhirl::MotorExecutionDomain executor(makeControl(&backend), &commands,
                                           &observations);
  assert(executor.begin(0U));
  assert(commands.publishTarget(10.0F, 100U) == 1U);
  executor.service(150U);

  const auto snapshot = readObservation(&observations);
  assert(executor.backendFaultLatched());
  assert(snapshot.backend_faulted);
  assert(!snapshot.actuator_enabled);
  assert(snapshot.applied_command_generation == 0U);
}

void testStuckEnabledIsVisibleAsHardBackendFailure() {
  FakeBackend backend{};
  triwhirl::MotorCommandMailbox commands;
  triwhirl::MotorObservationMailbox observations;
  triwhirl::MotorExecutionDomain executor(makeControl(&backend), &commands,
                                           &observations);
  assert(executor.begin(0U));
  assert(commands.publishTarget(10.0F, 100U) == 1U);
  executor.service(120U);
  assert(backend.actuator_enabled);

  backend.force_stuck_enabled = true;
  assert(commands.publishStop(130U) == 2U);
  executor.service(140U);
  const auto snapshot = readObservation(&observations);
  assert(executor.backendFaultLatched());
  assert(snapshot.backend_faulted);
  assert(snapshot.actuator_enabled);
}

void testUint32TimestampWrapKeepsLatencyMeasurable() {
  FakeBackend backend{};
  triwhirl::MotorCommandMailbox commands;
  triwhirl::MotorObservationMailbox observations;
  triwhirl::MotorExecutionDomain executor(makeControl(&backend), &commands,
                                           &observations);
  assert(executor.begin(0U));

  constexpr uint32_t issued = 0xFFFFFFF0U;
  constexpr uint32_t serviced = 0x00000020U;
  assert(commands.publishTarget(4.0F, issued) == 1U);
  executor.service(serviced);
  const auto snapshot = readObservation(&observations);
  assert(snapshot.command_apply_latency_us == 48U);
}

void testBeginFailurePublishesFaultedSnapshot() {
  FakeBackend backend{};
  backend.begin_result = false;
  triwhirl::MotorCommandMailbox commands;
  triwhirl::MotorObservationMailbox observations;
  triwhirl::MotorExecutionDomain executor(makeControl(&backend), &commands,
                                           &observations);
  assert(!executor.begin(77U));
  const auto snapshot = readObservation(&observations);
  assert(!snapshot.initialized);
  assert(!snapshot.sensor_valid);
  assert(snapshot.backend_faulted);
  assert(!snapshot.actuator_enabled);
  assert(snapshot.serviced_at_us32 == 77U);
}

}  // namespace

int main() {
  testBeginStartsDeenergized();
  testTargetAndStopAreAppliedOnMotorService();
  testLatestCommandWinsWithoutQueueBacklog();
  testCommandTimeoutStopsInOneServiceInvocation();
  testAlreadyExpiredNewTargetNeverEnergizes();
  testSensorInvalidityForcesSameInvocationStop();
  testBackendFaultLatchesAndPreventsReenable();
  testBackendCommandRejectionFailsClosed();
  testStuckEnabledIsVisibleAsHardBackendFailure();
  testUint32TimestampWrapKeepsLatencyMeasurable();
  testBeginFailurePublishesFaultedSnapshot();
  return 0;
}
