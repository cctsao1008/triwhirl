#include "simplefoc_motor_control.hpp"

#include <cmath>

namespace triwhirl::simplefoc_compat {

MotorControl SimpleFocMotorControlBackend::makeControl(
    const float target_velocity_limit_rad_s) {
  MotorControlOps ops{};
  ops.begin = &SimpleFocMotorControlBackend::beginThunk;
  ops.service_backend = &SimpleFocMotorControlBackend::serviceBackendThunk;
  ops.command_target_velocity =
      &SimpleFocMotorControlBackend::commandTargetVelocityThunk;
  ops.stop = &SimpleFocMotorControlBackend::stopThunk;
  ops.observation = &SimpleFocMotorControlBackend::observationThunk;
  return MotorControl(this, ops, target_velocity_limit_rad_s);
}

bool SimpleFocMotorControlBackend::beginThunk(void* context) {
  return static_cast<SimpleFocMotorControlBackend*>(context)->beginImpl();
}

void SimpleFocMotorControlBackend::serviceBackendThunk(void* context) {
  static_cast<SimpleFocMotorControlBackend*>(context)->serviceBackendImpl();
}

bool SimpleFocMotorControlBackend::commandTargetVelocityThunk(
    void* context, const float target_velocity_rad_s) {
  return static_cast<SimpleFocMotorControlBackend*>(context)
      ->commandTargetVelocityImpl(target_velocity_rad_s);
}

void SimpleFocMotorControlBackend::stopThunk(void* context) {
  static_cast<SimpleFocMotorControlBackend*>(context)->stopImpl();
}

MotorControlObservation SimpleFocMotorControlBackend::observationThunk(
    void* context) {
  return static_cast<SimpleFocMotorControlBackend*>(context)->observationImpl();
}

bool SimpleFocMotorControlBackend::beginImpl() {
  // All values supplied here are compile-probe inputs.  This adapter does not
  // declare them to be valid TriWhirl commissioning parameters.
  encoder_bus_.begin(sda_gpio_, scl_gpio_, i2c_hz_);
  sensor_.init(&encoder_bus_);
  motor_.linkSensor(&sensor_);

  driver_.voltage_power_supply = supply_voltage_v_;
  driver_.init();
  motor_.linkDriver(&driver_);

  motor_.torque_controller = TorqueControlType::voltage;
  motor_.controller = MotionControlType::velocity;
  motor_.init();
  motor_.initFOC();

  // Start the adapter in the de-energized command state.  SimpleFOC calls stay
  // in the motor domain; the attitude-facing stop() callback only requests this
  // state and never performs driver work itself.
  motor_.disable();
  initialized_ = true;
  command_enabled_ = false;
  motor_enabled_ = false;
  target_velocity_rad_s_ = 0.0F;
  observation_ = {};
  observation_.initialized = true;
  return true;
}

void SimpleFocMotorControlBackend::serviceBackendImpl() {
  if (!initialized_) {
    return;
  }

  // This is the only adapter operation that enters SimpleFOC.  In the future
  // production runtime it belongs to the motor execution domain, not the
  // attitude-control task.
  if (!command_enabled_) {
    if (motor_enabled_) {
      motor_.disable();
      motor_enabled_ = false;
    }
  } else {
    if (!motor_enabled_) {
      motor_.enable();
      motor_enabled_ = true;
    }
    motor_.loopFOC();
    motor_.move(target_velocity_rad_s_);
  }

  // Shaft observation remains available while the actuator is disabled.  Any
  // AS5600 access performed here is still confined to the motor domain.
  const float shaft_velocity_rad_s = motor_.shaftVelocity();
  const bool velocity_valid = std::isfinite(shaft_velocity_rad_s);

  observation_.initialized = true;
  observation_.sensor_valid = velocity_valid;
  observation_.command_enabled = command_enabled_;
  observation_.backend_faulted = !velocity_valid;
  observation_.shaft_velocity_rad_s =
      velocity_valid ? shaft_velocity_rad_s : 0.0F;
  observation_.target_velocity_rad_s =
      command_enabled_ ? target_velocity_rad_s_ : 0.0F;
}

bool SimpleFocMotorControlBackend::commandTargetVelocityImpl(
    const float target_velocity_rad_s) {
  if (!initialized_ || !std::isfinite(target_velocity_rad_s)) {
    return false;
  }
  target_velocity_rad_s_ = target_velocity_rad_s;
  command_enabled_ = true;
  return true;
}

void SimpleFocMotorControlBackend::stopImpl() {
  // Do not call SimpleFOC here.  The command side is deliberately a cached,
  // non-blocking request.  The motor domain de-energizes the actuator on its
  // next service cycle rather than interpreting stop as a zero-speed hold.
  target_velocity_rad_s_ = 0.0F;
  command_enabled_ = false;
}

MotorControlObservation SimpleFocMotorControlBackend::observationImpl() const {
  return observation_;
}

}  // namespace triwhirl::simplefoc_compat
