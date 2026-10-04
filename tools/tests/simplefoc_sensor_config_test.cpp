#include <cassert>

#include "triwhirl/simplefoc_motor_backend.hpp"
#include "triwhirl/simplefoc_sensor_path.hpp"

int main() {
  using triwhirl::simplefoc::SimpleFocMotorBackendConfig;
  using triwhirl::simplefoc::SimpleFocSensorConfig;
  using triwhirl::simplefoc::simpleFocSensorConfigFromMotorBackendConfig;
  using triwhirl::simplefoc::validSimpleFocMotorBackendConfig;
  using triwhirl::simplefoc::validSimpleFocSensorConfig;

  // Passive sensor commissioning is invalid by default.
  SimpleFocSensorConfig sensor{};
  assert(!validSimpleFocSensorConfig(sensor));

  sensor.i2c_bus_index = 0;
  sensor.sda_gpio = 23;
  sensor.scl_gpio = 5;
  sensor.i2c_hz = 400000U;
  assert(validSimpleFocSensorConfig(sensor));

  // The sensor stage is independent of all motor/FOC tuning fields.
  SimpleFocMotorBackendConfig motor{};
  motor.i2c_bus_index = sensor.i2c_bus_index;
  motor.sda_gpio = sensor.sda_gpio;
  motor.scl_gpio = sensor.scl_gpio;
  motor.i2c_hz = sensor.i2c_hz;
  assert(validSimpleFocSensorConfig(
      simpleFocSensorConfigFromMotorBackendConfig(motor)));
  assert(!validSimpleFocMotorBackendConfig(motor));

  // Validate the complete motor config independently.
  motor.pole_pairs = 1;
  motor.pwm_a_gpio = 33;
  motor.pwm_b_gpio = 25;
  motor.pwm_c_gpio = 32;
  motor.supply_voltage_v = 5.0F;
  motor.voltage_limit_v = 1.0F;
  motor.sensor_align_voltage_v = 0.5F;
  motor.target_velocity_limit_rad_s = 10.0F;
  motor.velocity_p = 0.0F;
  motor.velocity_i = 0.0F;
  motor.velocity_d = 0.0F;
  motor.velocity_output_ramp = 0.0F;
  motor.velocity_lpf_tf_s = 0.02F;
  assert(validSimpleFocMotorBackendConfig(motor));

  // Sensor validation rejects illegal bus/pin/rate combinations without
  // depending on motor parameters.
  SimpleFocSensorConfig invalid = sensor;
  invalid.i2c_bus_index = 2;
  assert(!validSimpleFocSensorConfig(invalid));
  invalid = sensor;
  invalid.scl_gpio = invalid.sda_gpio;
  assert(!validSimpleFocSensorConfig(invalid));
  invalid = sensor;
  invalid.i2c_hz = 0U;
  assert(!validSimpleFocSensorConfig(invalid));

  // Full backend validation still rejects bad motor-only values even when its
  // reusable sensor sub-config is valid.
  SimpleFocMotorBackendConfig invalid_motor = motor;
  invalid_motor.pole_pairs = 0;
  assert(!validSimpleFocMotorBackendConfig(invalid_motor));
  invalid_motor = motor;
  invalid_motor.pwm_c_gpio = invalid_motor.pwm_b_gpio;
  assert(!validSimpleFocMotorBackendConfig(invalid_motor));
  invalid_motor = motor;
  invalid_motor.voltage_limit_v = 6.0F;
  assert(!validSimpleFocMotorBackendConfig(invalid_motor));

  return 0;
}
