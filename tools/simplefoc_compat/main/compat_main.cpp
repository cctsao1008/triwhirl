#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

#include "simplefoc_motor_control.hpp"
#include "triwhirl/motor_control.hpp"

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
constexpr unsigned long kProbeI2cHz = 400000UL;
constexpr float kProbeSupplyV = 5.0F;
constexpr float kProbeTargetVelocityLimitRadS = 80.0F;

TwoWire encoder_bus(1);
MagneticSensorI2C sensor(AS5600_I2C);
BLDCMotor motor(kProbePolePairs);
BLDCDriver3PWM driver(kProbePwmA, kProbePwmB, kProbePwmC);

triwhirl::simplefoc_compat::SimpleFocMotorControlBackend motor_backend(
    encoder_bus, sensor, motor, driver, kProbeSda, kProbeScl, kProbeI2cHz,
    kProbeSupplyV);
triwhirl::MotorControl motor_control =
    motor_backend.makeControl(kProbeTargetVelocityLimitRadS);

void exerciseMotorControlBoundary() {
  if (!motor_control.begin()) {
    return;
  }

  // The future attitude task is allowed to issue only a bounded mechanical
  // target and read a cached mechanical snapshot.  It does not touch SimpleFOC
  // objects, AS5600, electrical angle, phase voltage, or PWM state.
  (void)motor_control.commandTargetVelocityRadS(0.0F);

  // Compile-only representation of the separate motor execution domain.
  motor_control.serviceBackend();
  const triwhirl::MotorControlObservation observation =
      motor_control.observation();
  (void)observation;

  motor_control.stop();
  motor_control.serviceBackend();
}

}  // namespace

void setup() {
  exerciseMotorControlBoundary();
}

void loop() {
  // Compile/link probe only. Runtime scheduling and hardware behavior are
  // deliberately out of scope.
}
