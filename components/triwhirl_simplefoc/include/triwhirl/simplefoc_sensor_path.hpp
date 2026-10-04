#pragma once

#include <cmath>
#include <cstdint>

#if defined(TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND)
#include <Wire.h>
#include <sensors/MagneticSensorI2C.h>
#endif

namespace triwhirl::simplefoc {

// Minimal configuration for passive AS5600 commissioning. No motor, driver,
// voltage, pole-pair or velocity-loop value is required to validate this stage.
struct SimpleFocSensorConfig {
  int i2c_bus_index = -1;
  int sda_gpio = -1;
  int scl_gpio = -1;
  std::uint32_t i2c_hz = 0U;
};

inline bool validSimpleFocSensorConfig(const SimpleFocSensorConfig& config) {
  const bool valid_bus = config.i2c_bus_index == 0 || config.i2c_bus_index == 1;
  const bool valid_pins = config.sda_gpio >= 0 && config.scl_gpio >= 0 &&
                          config.sda_gpio != config.scl_gpio;
  return valid_bus && valid_pins && config.i2c_hz > 0U;
}

struct SimpleFocSensorObservation {
  bool initialized = false;
  bool transport_valid = false;
  bool sample_valid = false;
  std::uint8_t wire_error = 0xFFU;
  float shaft_angle_rad = 0.0F;
  float shaft_velocity_rad_s = 0.0F;
};

#if defined(TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND)

class SimpleFocMotorBackend;

// SimpleFOC-owned passive sensor path. This class contains no motor driver and
// cannot initialize PWM/FOC. It uses the exact upstream MagneticSensorI2C +
// Sensor update/angle/velocity implementation that the full motor backend later
// links into BLDCMotor.
class SimpleFocSensorPath {
 public:
  explicit SimpleFocSensorPath(const SimpleFocSensorConfig& config)
      : config_(config), encoder_bus_(config.i2c_bus_index), sensor_(AS5600_I2C) {}

  SimpleFocSensorPath(const SimpleFocSensorPath&) = delete;
  SimpleFocSensorPath& operator=(const SimpleFocSensorPath&) = delete;

  bool configValid() const { return validSimpleFocSensorConfig(config_); }

  // Starts only the Arduino TwoWire + SimpleFOC MagneticSensorI2C path.
  // This performs no motor-driver, PWM, BLDCMotor or FOC initialization.
  bool begin();

  // Refreshes the SimpleFOC sensor state and cached mechanical observation.
  // Returns true only when the most recent sample is finite and the upstream
  // MagneticSensorI2C transport status reports success.
  bool service();

  bool initialized() const { return initialized_; }
  const SimpleFocSensorObservation& observation() const { return observation_; }

 private:
  friend class SimpleFocMotorBackend;

  // The full backend is the only consumer allowed to obtain the exact sensor
  // object for BLDCMotor::linkSensor(). Keeping this private prevents the
  // attitude/system layer from acquiring a SimpleFOC electrical dependency.
  MagneticSensorI2C* motorSensorHandle() { return &sensor_; }

  SimpleFocSensorConfig config_{};
  TwoWire encoder_bus_;
  MagneticSensorI2C sensor_;
  bool initialized_ = false;
  SimpleFocSensorObservation observation_{};
};

#endif  // TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND

}  // namespace triwhirl::simplefoc
