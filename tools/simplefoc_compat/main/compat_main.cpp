#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

namespace {

// Compile-only placeholders. This target is never flashed and these values are
// not hardware authority for TriWhirl. Real board/motor parameters must come
// from schematic/datasheet evidence or independent measurement.
constexpr int kProbePolePairs = 1;
constexpr int kProbePwmA = 2;
constexpr int kProbePwmB = 4;
constexpr int kProbePwmC = 16;
constexpr int kProbeSda = 21;
constexpr int kProbeScl = 22;
constexpr float kProbeSupplyV = 5.0F;

TwoWire encoder_bus(1);
MagneticSensorI2C sensor(AS5600_I2C);
BLDCMotor motor(kProbePolePairs);
BLDCDriver3PWM driver(kProbePwmA, kProbePwmB, kProbePwmC);

void exerciseLatestStableSimpleFocApi() {
  // AS5600 and shaft-velocity estimation belong to SimpleFOC.
  encoder_bus.begin(kProbeSda, kProbeScl, 400000);
  sensor.init(&encoder_bus);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = kProbeSupplyV;
  driver.init();
  motor.linkDriver(&driver);

  // The system-level controller always commands wheel target velocity.
  // Torque/current/voltage realization remains internal to SimpleFOC.
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity;

  motor.init();
  motor.initFOC();
  motor.loopFOC();

  const float wheel_rate_rad_s = motor.shaftVelocity();
  const float target_velocity_rad_s = wheel_rate_rad_s;
  motor.move(target_velocity_rad_s);
}

}  // namespace

extern "C" void app_main(void) {
  initArduino();
  exerciseLatestStableSimpleFocApi();
}
