#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

namespace {

// Isolated upstream API/toolchain probe only. These values are compile-time
// placeholders and are not hardware authority for TriWhirl.
constexpr int kProbePolePairs = 1;
constexpr int kProbePwmA = 2;
constexpr int kProbePwmB = 4;
constexpr int kProbePwmC = 16;
constexpr float kProbeSupplyV = 5.0F;
constexpr float kProbeTargetVelocityRadS = 0.0F;

MagneticSensorI2C sensor(AS5600_I2C);
BLDCMotor motor(kProbePolePairs);
BLDCDriver3PWM driver(kProbePwmA, kProbePwmB, kProbePwmC);

void exerciseRawSimpleFocApi() {
  sensor.init(&Wire);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = kProbeSupplyV;
  driver.init();
  motor.linkDriver(&driver);

  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity;
  motor.init();
  motor.initFOC();
  motor.enable();
  motor.loopFOC();
  motor.move(kProbeTargetVelocityRadS);
  volatile float shaft_velocity_rad_s = motor.shaftVelocity();
  (void)shaft_velocity_rad_s;
  motor.disable();
}

}  // namespace

void setup() {
  // This project is CI compile/link tooling and must not be flashed. Keeping the
  // API path reachable from setup prevents dead-code elimination from turning
  // the compatibility job into a dependency-download-only check.
  exerciseRawSimpleFocApi();
}

void loop() {}
