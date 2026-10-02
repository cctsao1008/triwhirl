#include <cassert>
#include <limits>

#include "triwhirl/simplefoc_motor_backend.hpp"

namespace {

triwhirl::simplefoc::SimpleFocMotorBackendConfig validConfig() {
  triwhirl::simplefoc::SimpleFocMotorBackendConfig config{};
  config.i2c_bus_index = 1;
  config.sda_gpio = 21;
  config.scl_gpio = 22;
  config.i2c_hz = 400000U;
  config.pole_pairs = 7;
  config.pwm_a_gpio = 2;
  config.pwm_b_gpio = 4;
  config.pwm_c_gpio = 16;
  config.supply_voltage_v = 12.0F;
  config.voltage_limit_v = 3.0F;
  config.sensor_align_voltage_v = 1.0F;
  config.target_velocity_limit_rad_s = 50.0F;
  config.velocity_p = 0.1F;
  config.velocity_i = 0.2F;
  config.velocity_d = 0.0F;
  config.velocity_output_ramp = 100.0F;
  config.velocity_lpf_tf_s = 0.02F;
  return config;
}

}  // namespace

int main() {
  using triwhirl::simplefoc::SimpleFocMotorBackendConfig;
  using triwhirl::simplefoc::validSimpleFocMotorBackendConfig;

  // The default object must be unusable: no commissioning parameter is allowed
  // to become an accidental hardware default.
  const SimpleFocMotorBackendConfig defaults{};
  assert(!validSimpleFocMotorBackendConfig(defaults));

  const SimpleFocMotorBackendConfig valid = validConfig();
  assert(validSimpleFocMotorBackendConfig(valid));

  auto invalid = valid;
  invalid.i2c_bus_index = -1;
  assert(!validSimpleFocMotorBackendConfig(invalid));

  invalid = valid;
  invalid.sda_gpio = invalid.scl_gpio;
  assert(!validSimpleFocMotorBackendConfig(invalid));

  invalid = valid;
  invalid.pwm_c_gpio = invalid.pwm_a_gpio;
  assert(!validSimpleFocMotorBackendConfig(invalid));

  invalid = valid;
  invalid.pole_pairs = 0;
  assert(!validSimpleFocMotorBackendConfig(invalid));

  invalid = valid;
  invalid.voltage_limit_v = invalid.supply_voltage_v + 0.1F;
  assert(!validSimpleFocMotorBackendConfig(invalid));

  invalid = valid;
  invalid.sensor_align_voltage_v = invalid.supply_voltage_v + 0.1F;
  assert(!validSimpleFocMotorBackendConfig(invalid));

  invalid = valid;
  invalid.target_velocity_limit_rad_s = 0.0F;
  assert(!validSimpleFocMotorBackendConfig(invalid));

  invalid = valid;
  invalid.velocity_p = -0.1F;
  assert(!validSimpleFocMotorBackendConfig(invalid));

  invalid = valid;
  invalid.velocity_d = std::numeric_limits<float>::quiet_NaN();
  assert(!validSimpleFocMotorBackendConfig(invalid));

  invalid = valid;
  invalid.velocity_lpf_tf_s = 0.0F;
  assert(!validSimpleFocMotorBackendConfig(invalid));

  return 0;
}
