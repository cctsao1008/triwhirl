#include "runtime_sensor_state.hpp"

#include "runtime_egress.hpp"
#include "runtime_state.hpp"

namespace triwhirl::runtime {

void commitRuntimeImuSample(const triwhirl::drivers::Mpu6050Sample& sample) {
  using namespace triwhirl::runtime::state;

  imu_sample = sample;
  imu_sample_valid = true;
  if (!gyro_calibration.active) {
    return;
  }

  for (int axis = 0; axis < 3; ++axis) {
    gyro_calibration.sum_rad_s[axis] +=
        static_cast<double>(sample.gyro_rad_s[axis]);
  }
  ++gyro_calibration.collected_samples;
  if (gyro_calibration.collected_samples < gyro_calibration.target_samples) {
    return;
  }

  const double denominator =
      static_cast<double>(gyro_calibration.collected_samples);
  for (int axis = 0; axis < 3; ++axis) {
    gyro_bias_rad_s[axis] = static_cast<float>(
        gyro_calibration.sum_rad_s[axis] / denominator);
  }
  gyro_calibration.active = false;
  gyro_bias_valid = true;

  RuntimeStateEvent event{};
  event.type = RuntimeStateEventType::kImuCalibrationComplete;
  event.float0 = gyro_bias_rad_s[0];
  event.float1 = gyro_bias_rad_s[1];
  event.float2 = gyro_bias_rad_s[2];
  publishRuntimeStateEvent(event);
}

}  // namespace triwhirl::runtime
