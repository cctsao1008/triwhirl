#pragma once

#include <cstdint>

namespace triwhirl::runtime {

struct RuntimeSnapshot {
  std::uint32_t t_us = 0U;

  bool attitude_initialized = false;
  bool attitude_valid = false;
  float attitude_angle_rad = 0.0F;
  float attitude_rate_rad_s = 0.0F;
  float attitude_residual_bias_rad_s = 0.0F;
  float attitude_innovation = 0.0F;
  float attitude_accel_weight = 0.0F;
  float wheel_rate_rad_s = 0.0F;

  bool safety_faulted = false;
  std::uint32_t safety_fault_mask = 0U;
  std::uint32_t safety_first_fault = 0U;

  bool telemetry_enabled = false;
  bool swing_active = false;

  std::uint32_t timing_target_us = 0U;
  std::uint32_t timing_hard_period_us = 0U;
  std::uint64_t timing_iterations = 0U;
  std::uint32_t timing_last_exec_us = 0U;
  std::uint32_t timing_max_exec_us = 0U;
  std::uint32_t timing_min_period_us = 0U;
  std::uint32_t timing_max_period_us = 0U;
  std::uint64_t timing_overruns = 0U;
  std::uint64_t timing_late_periods = 0U;
  std::uint32_t uart_tx_drop_bytes = 0U;

  // Realtime-owned state copied into the same bounded snapshot so Core 0 can
  // answer aggregate status and IMU diagnostics without touching live state.
  std::uint8_t motor_mode = 0U;
  float motor_vq_v = 0.0F;
  float motor_electrical_hz = 0.0F;
  float motor_amplitude_v = 0.0F;
  bool motor_config_valid = false;
  int motor_pole_pairs = 0;
  int motor_sensor_direction = 0;
  float motor_offset_rad = 0.0F;
  float motor_electrical_angle_rad = 0.0F;

  bool encoder_sample_valid = false;
  std::uint16_t encoder_raw_count = 0U;
  std::int64_t encoder_unwrapped_count = 0;
  float encoder_angle_rad = 0.0F;
  float encoder_unwrapped_rad = 0.0F;
  float encoder_velocity_rad_s = 0.0F;
  float encoder_instantaneous_velocity_rad_s = 0.0F;
  bool encoder_velocity_valid = false;
  std::uint32_t encoder_read_errors = 0U;

  bool imu_ready = false;
  bool imu_sample_valid = false;
  bool imu_identity_valid = false;
  std::uint8_t imu_who_am_i = 0U;
  bool imu_bias_valid = false;
  bool imu_calibrating = false;
  float imu_ax_mps2 = 0.0F;
  float imu_ay_mps2 = 0.0F;
  float imu_az_mps2 = 0.0F;
  float imu_gx_rad_s = 0.0F;
  float imu_gy_rad_s = 0.0F;
  float imu_gz_rad_s = 0.0F;
  float imu_temperature_c = 0.0F;
  float imu_bias_x_rad_s = 0.0F;
  float imu_bias_y_rad_s = 0.0F;
  float imu_bias_z_rad_s = 0.0F;
  int imu_map_sin_axis = 0;
  int imu_map_cos_axis = 1;
  int imu_map_gyro_axis = 2;
  int imu_map_sin_sign = 1;
  int imu_map_cos_sign = 1;
  int imu_map_gyro_sign = 1;
  std::uint32_t imu_read_errors = 0U;

  // Logger status is supervisory data. The diagnostic publisher refreshes this
  // cached view at a lower rate than the 1 kHz control snapshot publication.
  std::uint8_t log_state = 0U;
  std::uint32_t log_partition_bytes = 0U;
  std::uint32_t log_prepared_bytes = 0U;
  std::uint32_t log_max_records = 0U;
  std::uint32_t log_buffered_bytes = 0U;
  std::uint32_t log_records_written = 0U;
  std::uint32_t log_dropped_records = 0U;
  std::uint32_t log_logical_bytes = 0U;
  bool log_flash_writes_allowed = true;
  bool log_critical_window = false;
  bool log_dump_active = false;
};

// Single-writer (Core 1) bounded publication. The implementation keeps only
// the latest complete snapshot; publishing never blocks the realtime task.
bool initRuntimeSnapshotChannel();
void publishRuntimeSnapshot(const RuntimeSnapshot& snapshot);

// Non-blocking read for supervisory code. Returns the latest complete snapshot
// without consuming it.
bool readLatestRuntimeSnapshot(RuntimeSnapshot* snapshot);

}  // namespace triwhirl::runtime
