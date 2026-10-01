#include <cassert>
#include <cmath>
#include <limits>

#include "triwhirl/motor_control.hpp"

namespace {

struct FakeBackend {
  bool begin_result = true;
  bool command_result = true;
  bool began = false;
  bool stopped = false;
  int service_calls = 0;
  int command_calls = 0;
  float last_target_rad_s = 0.0F;
  triwhirl::MotorControlObservation observation{};
};

bool begin(void* context) {
  auto* backend = static_cast<FakeBackend*>(context);
  backend->began = true;
  return backend->begin_result;
}

void serviceBackend(void* context) {
  ++static_cast<FakeBackend*>(context)->service_calls;
}

bool commandTargetVelocity(void* context, const float target_rad_s) {
  auto* backend = static_cast<FakeBackend*>(context);
  ++backend->command_calls;
  backend->last_target_rad_s = target_rad_s;
  return backend->command_result;
}

void stop(void* context) {
  static_cast<FakeBackend*>(context)->stopped = true;
}

triwhirl::MotorControlObservation observation(void* context) {
  return static_cast<FakeBackend*>(context)->observation;
}

triwhirl::MotorControl makeControl(FakeBackend* backend, const float limit) {
  triwhirl::MotorControlOps ops{};
  ops.begin = begin;
  ops.service_backend = serviceBackend;
  ops.command_target_velocity = commandTargetVelocity;
  ops.stop = stop;
  ops.observation = observation;
  return triwhirl::MotorControl(backend, ops, limit);
}

}  // namespace

int main() {
  FakeBackend backend{};
  backend.observation.initialized = true;
  backend.observation.sensor_valid = true;
  backend.observation.command_enabled = true;
  backend.observation.actuator_enabled = true;
  backend.observation.shaft_velocity_rad_s = 12.5F;
  backend.observation.target_velocity_rad_s = -7.0F;

  const triwhirl::MotorControl control = makeControl(&backend, 80.0F);
  assert(control.valid());
  assert(control.begin());
  assert(backend.began);

  control.serviceBackend();
  assert(backend.service_calls == 1);

  assert(control.commandTargetVelocityRadS(79.5F));
  assert(backend.command_calls == 1);
  assert(std::fabs(backend.last_target_rad_s - 79.5F) < 1.0e-6F);

  // The boundary rejects invalid and out-of-envelope commands before the
  // backend sees them.
  assert(!control.commandTargetVelocityRadS(80.01F));
  assert(!control.commandTargetVelocityRadS(-80.01F));
  assert(!control.commandTargetVelocityRadS(
      std::numeric_limits<float>::quiet_NaN()));
  assert(!control.commandTargetVelocityRadS(
      std::numeric_limits<float>::infinity()));
  assert(backend.command_calls == 1);

  const auto snapshot = control.observation();
  assert(snapshot.initialized);
  assert(snapshot.sensor_valid);
  assert(snapshot.command_enabled);
  assert(snapshot.actuator_enabled);
  assert(!snapshot.backend_faulted);
  assert(std::fabs(snapshot.shaft_velocity_rad_s - 12.5F) < 1.0e-6F);
  assert(std::fabs(snapshot.target_velocity_rad_s + 7.0F) < 1.0e-6F);

  control.stop();
  assert(backend.stopped);

  // Invalid bindings fail closed and produce a zero/default snapshot.
  const triwhirl::MotorControl invalid{};
  assert(!invalid.valid());
  assert(!invalid.begin());
  assert(!invalid.commandTargetVelocityRadS(0.0F));
  const auto empty = invalid.observation();
  assert(!empty.initialized);
  assert(!empty.sensor_valid);
  assert(!empty.actuator_enabled);

  const triwhirl::MotorControl invalid_limit = makeControl(&backend, 0.0F);
  assert(!invalid_limit.valid());

  return 0;
}
