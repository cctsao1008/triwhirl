#pragma once

#include <cstdint>
#include <type_traits>

namespace triwhirl::runtime {

struct RuntimeTelemetryFrame {
  std::uint64_t timing_overruns = 0U;
  std::int64_t encoder_unwrapped_count = 0;

  float motor_vq_v = 0.0F;
  float motor_electrical_angle_rad = 0.0F;
  float motor_electrical_hz = 0.0F;
  float encoder_angle_rad = 0.0F;
  float encoder_unwrapped_rad = 0.0F;
  float encoder_velocity_rad_s = 0.0F;
  float encoder_instantaneous_velocity_rad_s = 0.0F;
  float imu_ax_mps2 = 0.0F;
  float imu_ay_mps2 = 0.0F;
  float imu_az_mps2 = 0.0F;
  float imu_gx_rad_s = 0.0F;
  float imu_gy_rad_s = 0.0F;
  float imu_gz_rad_s = 0.0F;
  float attitude_angle_rad = 0.0F;
  float attitude_rate_rad_s = 0.0F;
  float attitude_accel_weight = 0.0F;

  std::uint32_t t_us = 0U;
  std::uint32_t encoder_read_errors = 0U;
  std::uint32_t imu_read_errors = 0U;
  std::uint32_t timing_last_exec_us = 0U;
  std::uint32_t timing_max_exec_us = 0U;
  std::uint32_t safety_fault_mask = 0U;

  std::uint16_t encoder_raw_count = 0U;
  std::uint8_t motor_mode = 0U;
  std::uint8_t encoder_status_valid = 0U;
  std::uint8_t encoder_sample_valid = 0U;
  std::uint8_t encoder_magnet_detected = 0U;
  std::uint8_t encoder_velocity_valid = 0U;
  std::uint8_t imu_sample_valid = 0U;
  std::uint8_t attitude_valid = 0U;
};

static_assert(std::is_trivially_copyable_v<RuntimeTelemetryFrame>,
              "RuntimeTelemetryFrame must remain trivially copyable");
static_assert(sizeof(RuntimeTelemetryFrame) <= 128U,
              "RuntimeTelemetryFrame grew beyond the bounded egress budget");

}  // namespace triwhirl::runtime
