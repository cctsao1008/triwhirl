#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

#include "triwhirl/motor_control.hpp"

#if defined(TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND)
// Keep the production-shaped backend coupled only to the upstream APIs it
// actually owns. Avoid SimpleFOC.h's umbrella sensor includes so Route-B does
// not acquire unrelated SPI/library header dependencies.
#include <BLDCMotor.h>
#include <Wire.h>
#include <drivers/BLDCDriver3PWM.h>
#include <sensors/MagneticSensorI2C.h>
#endif

namespace triwhirl::simplefoc {

// Explicit commissioning boundary for the production-shaped SimpleFOC owner.
// Every value that can affect motor/sensor behavior starts invalid so merely
// default-constructing this object can never produce a usable motor setup.
struct SimpleFocMotorBackendConfig {
  int i2c_bus_index = -1;
  int sda_gpio = -1;
  int scl_gpio = -1;
  std::uint32_t i2c_hz = 0U;

  int pole_pairs = 0;
  int pwm_a_gpio = -1;
  int pwm_b_gpio = -1;
  int pwm_c_gpio = -1;

  float supply_voltage_v = std::numeric_limits<float>::quiet_NaN();
  float voltage_limit_v = std::numeric_limits<float>::quiet_NaN();
  float sensor_align_voltage_v = std::numeric_limits<float>::quiet_NaN();
  float target_velocity_limit_rad_s =
      std::numeric_limits<float>::quiet_NaN();

  float velocity_p = std::numeric_limits<float>::quiet_NaN();
  float velocity_i = std::numeric_limits<float>::quiet_NaN();
  float velocity_d = std::numeric_limits<float>::quiet_NaN();
  float velocity_output_ramp = std::numeric_limits<float>::quiet_NaN();
  float velocity_lpf_tf_s = std::numeric_limits<float>::quiet_NaN();
};

inline bool validSimpleFocMotorBackendConfig(
    const SimpleFocMotorBackendConfig& config) {
  const auto finite_positive = [](const float value) {
    return std::isfinite(value) && value > 0.0F;
  };
  const auto finite_nonnegative = [](const float value) {
    return std::isfinite(value) && value >= 0.0F;
  };

  const bool valid_i2c_bus = config.i2c_bus_index == 0 ||
                             config.i2c_bus_index == 1;
  const bool valid_i2c_pins = config.sda_gpio >= 0 && config.scl_gpio >= 0 &&
                              config.sda_gpio != config.scl_gpio;
  const bool valid_pwm_pins =
      config.pwm_a_gpio >= 0 && config.pwm_b_gpio >= 0 &&
      config.pwm_c_gpio >= 0 && config.pwm_a_gpio != config.pwm_b_gpio &&
      config.pwm_a_gpio != config.pwm_c_gpio &&
      config.pwm_b_gpio != config.pwm_c_gpio;

  return valid_i2c_bus && valid_i2c_pins && config.i2c_hz > 0U &&
         config.pole_pairs > 0 && valid_pwm_pins &&
         finite_positive(config.supply_voltage_v) &&
         finite_positive(config.voltage_limit_v) &&
         config.voltage_limit_v <= config.supply_voltage_v &&
         finite_positive(config.sensor_align_voltage_v) &&
         config.sensor_align_voltage_v <= config.supply_voltage_v &&
         finite_positive(config.target_velocity_limit_rad_s) &&
         finite_nonnegative(config.velocity_p) &&
         finite_nonnegative(config.velocity_i) &&
         finite_nonnegative(config.velocity_d) &&
         finite_nonnegative(config.velocity_output_ramp) &&
         finite_positive(config.velocity_lpf_tf_s);
}

#if defined(TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND)

// Production-shaped Route-B backend. This class is intentionally unavailable
// to the native IDF-only build because that build has no Arduino/SimpleFOC
// dependency. The attitude layer still sees only triwhirl::MotorControl.
class SimpleFocMotorBackend {
 public:
  explicit SimpleFocMotorBackend(const SimpleFocMotorBackendConfig& config)
      : config_(config),
        encoder_bus_(config.i2c_bus_index),
        sensor_(AS5600_I2C),
        motor_(config.pole_pairs),
        driver_(config.pwm_a_gpio, config.pwm_b_gpio, config.pwm_c_gpio) {}

  SimpleFocMotorBackend(const SimpleFocMotorBackend&) = delete;
  SimpleFocMotorBackend& operator=(const SimpleFocMotorBackend&) = delete;

  bool configValid() const {
    return validSimpleFocMotorBackendConfig(config_);
  }

  MotorControl makeControl();

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
  void setFaulted();

  SimpleFocMotorBackendConfig config_{};
  TwoWire encoder_bus_;
  MagneticSensorI2C sensor_;
  BLDCMotor motor_;
  BLDCDriver3PWM driver_;

  bool initialized_ = false;
  bool command_enabled_ = false;
  bool motor_enabled_ = false;
  bool backend_faulted_ = false;
  float target_velocity_rad_s_ = 0.0F;
  MotorControlObservation observation_{};
};

#endif  // TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND

}  // namespace triwhirl::simplefoc
