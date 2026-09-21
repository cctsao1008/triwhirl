#pragma once

#include "triwhirl/drivers/mpu6050.hpp"

namespace triwhirl::runtime {

// Commits a completed Core-0 MPU6050 acquisition into the realtime-owned state.
// This performs no I2C and preserves gyro-calibration semantics.
void commitRuntimeImuSample(const triwhirl::drivers::Mpu6050Sample& sample);

}  // namespace triwhirl::runtime
