#include "triwhirl/simplefoc_motor_backend.hpp"

#if defined(TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND)

#include <cmath>

namespace triwhirl::simplefoc {

MotorControl SimpleFocMotorBackend::makeControl() {
  MotorControlOps ops{};
  ops.begin = &SimpleFocMotorBackend::beginThunk;
  ops.service_backend = &SimpleFocMotorBackend::serviceBackendThunk;
  ops.command_target_velocity = &SimpleFocMotorBackend::commandTargetVelocityThunk;
  ops.stop = &SimpleFocMotorBackend::stopThunk;
  ops.observation = &SimpleFocMotorBackend::observationThunk;
  return MotorControl(this, ops, config_.target_velocity_limit_rad_s);
}

bool SimpleFocMotorBackend::beginThunk(void* context) {
  return static_cast<SimpleFocMotorBackend*>(context)->beginImpl();
}

void SimpleFocMotorBackend::serviceBackendThunk(void* context) {
  static_cast<SimpleFocMotorBackend*>(context)->serviceBackendImpl();
}

bool SimpleFocMotorBackend::commandTargetVelocityThunk(
    void* context, const float target_velocity_rad_s) {
  return static_cast<SimpleFocMotorBackend*>(context)
      ->commandTargetVelocityImpl(target_velocity_rad_s);
}

void SimpleFocMotorBackend::stopThunk(void* context) {
  static_cast<SimpleFocMotorBackend*>(context)->stopImpl();
}

MotorControlObservation SimpleFocMotorBackend::observationThunk(void* context) {
  return static_cast<SimpleFocMotorBackend*>(context)->observationImpl();
}

void SimpleFocMotorBackend::setFaulted() {
  backend_faulted_ = true;
  command_enabled_ = false;
  target_velocity_rad_s_ = 0.0F;
  observation_.backend_faulted = true;
  observation_.command_enabled = false;
  observation_.target_velocity_rad_s = 0.0F;
}

bool SimpleFocMotorBackend::beginImpl() {
  if (initialized_) return !backend_faulted_;
  begin_failure_stage_ = SimpleFocMotorBeginFailureStage::kNone;
  if (!configValid()) {
    begin_failure_stage_ = SimpleFocMotorBeginFailureStage::kInvalidConfig;
    setFaulted();
    return false;
  }

  // Keep begin diagnostics as bounded in-memory state. The commissioning motor
  // task must not synchronously print/stream upstream debug output while it owns
  // the active motor path. A lower-priority command task exports the resulting
  // begin stage after the attempt completes.

  // Stage 1: initialize and prove the exact SimpleFOC-owned AS5600 path before
  // any motor driver/PWM/FOC initialization. The same MagneticSensorI2C object
  // is linked into BLDCMotor below; there is no parallel target-path reader.
  if (!sensor_path_.begin() || !sensor_path_.service() ||
      !sensor_path_.observation().sample_valid) {
    begin_failure_stage_ = SimpleFocMotorBeginFailureStage::kSensorPath;
    setFaulted();
    return false;
  }
  motor_.linkSensor(sensor_path_.motorSensorHandle());

  // Stage 2: only a fully valid motor config may progress into driver and FOC
  // initialization. Sensor-only commissioning never reaches any code below.
  driver_.voltage_power_supply = config_.supply_voltage_v;
  driver_.voltage_limit = config_.voltage_limit_v;
  if (driver_.init() == 0) {
    begin_failure_stage_ = SimpleFocMotorBeginFailureStage::kDriverInit;
    setFaulted();
    return false;
  }
  motor_.linkDriver(&driver_);

  // These are explicit TriWhirl backend choices, not inherited vendor values.
  // Motor-specific commissioning numbers all come from config_ and must be
  // supplied by the integration site before begin() can succeed.
  motor_.torque_controller = TorqueControlType::voltage;
  motor_.controller = MotionControlType::velocity;
  motor_.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor_.voltage_limit = config_.voltage_limit_v;
  motor_.voltage_sensor_align = config_.sensor_align_voltage_v;
  motor_.velocity_limit = config_.target_velocity_limit_rad_s;
  motor_.PID_velocity.P = config_.velocity_p;
  motor_.PID_velocity.I = config_.velocity_i;
  motor_.PID_velocity.D = config_.velocity_d;
  motor_.PID_velocity.output_ramp = config_.velocity_output_ramp;
  motor_.LPF_velocity.Tf = config_.velocity_lpf_tf_s;

  if (motor_.init() == 0) {
    begin_failure_stage_ = SimpleFocMotorBeginFailureStage::kMotorInit;
    setFaulted();
    return false;
  }
  if (motor_.initFOC() == 0) {
    begin_failure_stage_ = SimpleFocMotorBeginFailureStage::kInitFoc;
    motor_.disable();
    motor_enabled_ = false;
    setFaulted();
    return false;
  }

  motor_.disable();
  initialized_ = true;
  command_enabled_ = false;
  motor_enabled_ = false;
  backend_faulted_ = false;
  target_velocity_rad_s_ = 0.0F;
  begin_failure_stage_ = SimpleFocMotorBeginFailureStage::kNone;

  observation_ = {};
  observation_.initialized = true;
  observation_.sensor_valid = sensor_path_.observation().sample_valid;
  observation_.command_enabled = false;
  observation_.actuator_enabled = false;
  observation_.backend_faulted = false;
  observation_.shaft_velocity_rad_s =
      sensor_path_.observation().shaft_velocity_rad_s;
  return true;
}

void SimpleFocMotorBackend::serviceBackendImpl() {
  if (!initialized_ || backend_faulted_) return;

  if (!command_enabled_) {
    if (motor_enabled_) {
      motor_.disable();
      motor_enabled_ = false;
    }
    // Keep the SimpleFOC sensor state live even while the actuator is disabled.
    // This is the same sensor object linked into BLDCMotor, not a second reader.
    (void)sensor_path_.service();
  } else {
    if (!motor_enabled_) {
      motor_.enable();
      motor_enabled_ = true;
    }
    // SimpleFOC loopFOC() owns the active sensor update/electrical-angle path.
    motor_.loopFOC();
    motor_.move(target_velocity_rad_s_);
  }

  // shaftVelocity() remains the target mechanical wheel-state API. While the
  // motor is active loopFOC() has refreshed the linked sensor; while disabled
  // sensor_path_.service() refreshed the exact same SimpleFOC sensor object.
  const float shaft_velocity_rad_s = motor_.shaftVelocity();
  const bool velocity_valid = std::isfinite(shaft_velocity_rad_s);
  const bool transport_valid =
      sensor_path_.motorSensorHandle()->currWireError == 0U;
  const bool sensor_valid = velocity_valid && transport_valid;

  if (!velocity_valid) {
    if (motor_enabled_) {
      motor_.disable();
      motor_enabled_ = false;
    }
    setFaulted();
  }

  observation_.initialized = initialized_;
  observation_.sensor_valid = sensor_valid;
  observation_.command_enabled = command_enabled_;
  observation_.actuator_enabled = motor_enabled_;
  observation_.backend_faulted = backend_faulted_;
  observation_.shaft_velocity_rad_s =
      velocity_valid ? shaft_velocity_rad_s : 0.0F;
  observation_.target_velocity_rad_s =
      command_enabled_ ? target_velocity_rad_s_ : 0.0F;
}

bool SimpleFocMotorBackend::commandTargetVelocityImpl(
    const float target_velocity_rad_s) {
  if (!initialized_ || backend_faulted_ ||
      !std::isfinite(target_velocity_rad_s) ||
      std::fabs(target_velocity_rad_s) >
          config_.target_velocity_limit_rad_s) {
    return false;
  }

  target_velocity_rad_s_ = target_velocity_rad_s;
  command_enabled_ = true;
  return true;
}

void SimpleFocMotorBackend::stopImpl() {
  // MotorControl::stop() remains a bounded cached request. The motor execution
  // domain performs the actual SimpleFOC disable on its next service call.
  target_velocity_rad_s_ = 0.0F;
  command_enabled_ = false;
}

MotorControlObservation SimpleFocMotorBackend::observationImpl() const {
  return observation_;
}

}  // namespace triwhirl::simplefoc

#endif  // TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND
