#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

namespace {

// Source-aligned TRC-V1.1 motor/sensor topology. This target is compile-only:
// CI never flashes or executes it on hardware.
TwoWire encoder_bus(1);
MagneticSensorI2C sensor(AS5600_I2C);
BLDCMotor motor(7);
BLDCDriver3PWM driver(33, 25, 32);

void exerciseSourceAlignedApi() {
  // AS5600 stays inside the SimpleFOC sensor path.
  encoder_bus.begin(23, 5, 400000);
  sensor.init(&encoder_bus);
  motor.linkSensor(&sensor);

  // Preserve the proven vendor phase order and basic electrical limits.
  driver.voltage_power_supply = 8.3F;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_sensor_align = 3.0F;
  motor.voltage_limit = 4.0F;
  motor.velocity_limit = 140.0F;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // Vendor-aligned SimpleFOC velocity-loop commissioning profile.
  motor.PID_velocity.P = 0.035F;
  motor.PID_velocity.I = 0.8F;
  motor.LPF_velocity.Tf = 0.01F;

  // Prove both command interfaces needed by the corrected full-fuzzy design.
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::torque;

  motor.init();
  motor.initFOC();
  motor.loopFOC();
  motor.move(0.0F);  // swing path: voltage command in torque/voltage mode

  const float wheel_rate_rad_s = motor.shaftVelocity();
  motor.controller = MotionControlType::velocity;
  motor.move(wheel_rate_rad_s);  // balance path: rad/s target velocity
}

}  // namespace

extern "C" void app_main(void) {
  initArduino();
  exerciseSourceAlignedApi();
}
