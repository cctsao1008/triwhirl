#include "triwhirl/simplefoc_sensor_path.hpp"

#if defined(TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND)

#include <cmath>

namespace triwhirl::simplefoc {

bool SimpleFocSensorPath::begin() {
  if (initialized_) return true;
  if (!configValid()) return false;

  if (!encoder_bus_.begin(config_.sda_gpio, config_.scl_gpio, config_.i2c_hz)) {
    return false;
  }

  // MagneticSensorI2C::init() is the upstream SimpleFOC sensor initialization
  // path. It performs only I2C/sensor-state setup; this class has no motor
  // driver or PWM member that could be energized here.
  sensor_.init(&encoder_bus_);
  initialized_ = true;
  observation_ = {};
  observation_.initialized = true;

  // Prime the cached observation. A transport failure does not tear down the
  // I2C object so sensor-only commissioning can continue observing/retrying.
  (void)service();
  return true;
}

bool SimpleFocSensorPath::service() {
  if (!initialized_) return false;

  // One upstream sensor update performs the I2C angle read and advances
  // SimpleFOC's multi-turn/velocity state. getAngle()/getVelocity() below read
  // that cached state and do not introduce a TriWhirl-side encoder estimator.
  sensor_.update();
  const float angle_rad = sensor_.getAngle();
  const float velocity_rad_s = sensor_.getVelocity();
  const std::uint8_t wire_error = sensor_.currWireError;

  const bool transport_valid = wire_error == 0U;
  const bool sample_finite =
      std::isfinite(angle_rad) && std::isfinite(velocity_rad_s);

  observation_.initialized = true;
  observation_.transport_valid = transport_valid;
  observation_.sample_valid = transport_valid && sample_finite;
  observation_.wire_error = wire_error;
  observation_.shaft_angle_rad = sample_finite ? angle_rad : 0.0F;
  observation_.shaft_velocity_rad_s = sample_finite ? velocity_rad_s : 0.0F;
  return observation_.sample_valid;
}

}  // namespace triwhirl::simplefoc

#endif  // TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND
