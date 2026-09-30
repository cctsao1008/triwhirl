#pragma once

#include <SimpleFOC.h>
#include <Wire.h>

#include "triwhirl/motor_control.hpp"

namespace triwhirl::simplefoc_compat {

// Route-B compile/link proof for the target MotorControl boundary.
//
// The command/snapshot callbacks only touch cached scalar state.  The
// SimpleFOC calls and AS5600 access live in serviceBackend(), which represents
// the future motor execution domain and must not be invoked from the attitude
// control task merely to obtain a fresh velocity sample.
class SimpleFocMotorControlBackend {
 public:
  SimpleFocMotorControlBackend(TwoWire& encoder_bus,
                               MagneticSensorI2C& sensor,
                               BLDCMotor& motor,
                               BLDCDriver3PWM& driver,
                               int sda_gpio,
                               int scl_gpio,
                               unsigned long i2c_hz,
                               float supply_voltage_v)
      : encoder_bus_(encoder_bus),
        sensor_(sensor),
        motor_(motor),
        driver_(driver),
        sda_gpio_(sda_gpio),
        scl_gpio_(scl_gpio),
        i2c_hz_(i2c_hz),
        supply_voltage_v_(supply_voltage_v) {}

  MotorControl makeControl(float target_velocity_limit_rad_s);

 private:
  static bool beginThunk(void* context);
  static void serviceBackendThunk(void* context);
  static bool commandTargetVelocityThunk(void* context,
                                         float target_velocity_rad_s);
  static void stopThunk(void* context);
  static MotorControlObservation observationThunk(void* context);

  bool beginImpl();
  void serviceBackendImpl();
  bool commandTargetVelocityImpl(float target_velocity_rad_s);
  void stopImpl();
  MotorControlObservation observationImpl() const;

  TwoWire& encoder_bus_;
  MagneticSensorI2C& sensor_;
  BLDCMotor& motor_;
  BLDCDriver3PWM& driver_;
  int sda_gpio_;
  int scl_gpio_;
  unsigned long i2c_hz_;
  float supply_voltage_v_;

  bool initialized_ = false;
  bool command_enabled_ = false;
  float target_velocity_rad_s_ = 0.0F;
  MotorControlObservation observation_{};
};

}  // namespace triwhirl::simplefoc_compat
