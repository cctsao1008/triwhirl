// Issue #45 build/link spike only.
//
// This translation unit is compiled by the native ESP-IDF build as an empty
// file.  The Route-B PlatformIO environment defines
// TRIWHIRL_ROUTE_B_SIMPLEFOC_LINK_PROBE and asks the linker to retain the
// extern "C" symbol below.  That retained function references the real
// SimpleFOC AS5600 / velocity / FOC path, so a green Route-B build proves more
// than dependency download: the existing TriWhirl ESP-IDF component graph can
// actually compile and link against the pinned Arduino/SimpleFOC stack.
//
// The function is NEVER called.  Its pins, pole-pair count, voltage and target
// are compile-only placeholders and are not hardware authority.

#if defined(TRIWHIRL_ROUTE_B_SIMPLEFOC_LINK_PROBE)

#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>

extern "C" __attribute__((used, noinline))
void triwhirl_route_b_simplefoc_link_probe() {
  constexpr int kCompileOnlyPolePairs = 1;
  constexpr int kCompileOnlyPwmA = 2;
  constexpr int kCompileOnlyPwmB = 4;
  constexpr int kCompileOnlyPwmC = 16;
  constexpr float kCompileOnlySupplyV = 5.0F;
  constexpr float kCompileOnlyTargetVelocityRadS = 0.0F;

  MagneticSensorI2C sensor(AS5600_I2C);
  BLDCMotor motor(kCompileOnlyPolePairs);
  BLDCDriver3PWM driver(kCompileOnlyPwmA, kCompileOnlyPwmB,
                        kCompileOnlyPwmC);

  // These calls intentionally live in retained-but-unexecuted code.  They
  // force the Route-B linker to resolve the API surface that the production
  // motor domain will eventually need, without performing hardware I/O.
  sensor.init(&Wire);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = kCompileOnlySupplyV;
  driver.init();
  motor.linkDriver(&driver);

  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity;
  motor.init();
  motor.initFOC();
  motor.enable();
  motor.loopFOC();
  motor.move(kCompileOnlyTargetVelocityRadS);
  volatile float shaft_velocity_rad_s = motor.shaftVelocity();
  (void)shaft_velocity_rad_s;
  motor.disable();
}

#endif  // TRIWHIRL_ROUTE_B_SIMPLEFOC_LINK_PROBE
