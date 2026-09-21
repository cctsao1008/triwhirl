#include "runtime_diagnostics.hpp"

#include "runtime_snapshot.hpp"
#include "runtime_state.hpp"

namespace triwhirl::runtime {
namespace {

using namespace triwhirl::runtime::state;

constexpr std::uint32_t kLoggerSnapshotPeriodUs = 20000U;

bool imu_identity_checked = false;
bool imu_identity_valid = false;
std::uint8_t imu_identity = 0U;
LoggerStatus cached_logger_status{};
std::uint32_t last_logger_snapshot_us = 0U;

}  // namespace

void populateRuntimeDiagnosticSnapshot(RuntimeSnapshot* const snapshot) {
  if (snapshot == nullptr) {
    return;
  }

  snapshot->motor_mode = static_cast<std::uint8_t>(motor_mode);
  snapshot->motor_vq_v = vq_command_v;
  snapshot->motor_electrical_hz = open_loop_hz;
  snapshot->motor_amplitude_v = open_loop_amplitude_v;
  snapshot->motor_config_valid = motor_config_valid;
  snapshot->motor_pole_pairs = motor_config.pole_pairs;
  snapshot->motor_sensor_direction = motor_config.sensor_direction;
  snapshot->motor_offset_rad = motor_config.electrical_offset_rad;
  snapshot->motor_electrical_angle_rad = electrical_angle_rad;

  snapshot->encoder_sample_valid = encoder_sample_valid;
  snapshot->encoder_raw_count = wheel_state.raw_count;
  snapshot->encoder_unwrapped_count = wheel_state.unwrapped_count;
  snapshot->encoder_angle_rad = wheel_state.angle_rad;
  snapshot->encoder_unwrapped_rad = wheel_state.unwrapped_angle_rad;
  snapshot->encoder_velocity_rad_s = wheel_state.velocity_rad_s;
  snapshot->encoder_instantaneous_velocity_rad_s =
      wheel_state.instantaneous_velocity_rad_s;
  snapshot->encoder_velocity_valid = wheel_state.velocity_valid;
  snapshot->encoder_read_errors = encoder_read_errors;

  if (!imu_identity_checked) {
    imu_identity_checked = true;
    // If full initialization failed after WHO_AM_I succeeded, retain that
    // evidence instead of collapsing diagnostics to who=0x00. Both paths are
    // cached-only here; supervisor diagnostics never issue live MPU I2C.
    imu_identity_valid = imu_ready ? imu.readWhoAmI(&imu_identity)
                                   : imu.lastInitWhoAmI(&imu_identity);
  }

  snapshot->imu_ready = imu_ready;
  snapshot->imu_sample_valid = imu_sample_valid;
  snapshot->imu_identity_valid = imu_identity_valid;
  snapshot->imu_who_am_i = imu_identity;
  snapshot->imu_bias_valid = gyro_bias_valid;
  snapshot->imu_calibrating = gyro_calibration.active;
  snapshot->imu_ax_mps2 = imu_sample.accel_mps2[0];
  snapshot->imu_ay_mps2 = imu_sample.accel_mps2[1];
  snapshot->imu_az_mps2 = imu_sample.accel_mps2[2];
  snapshot->imu_gx_rad_s = correctedGyro(0);
  snapshot->imu_gy_rad_s = correctedGyro(1);
  snapshot->imu_gz_rad_s = correctedGyro(2);
  snapshot->imu_temperature_c = imu_sample.temperature_c;
  snapshot->imu_bias_x_rad_s = gyro_bias_rad_s[0];
  snapshot->imu_bias_y_rad_s = gyro_bias_rad_s[1];
  snapshot->imu_bias_z_rad_s = gyro_bias_rad_s[2];
  snapshot->imu_map_sin_axis = imu_map.accel_sin_axis;
  snapshot->imu_map_cos_axis = imu_map.accel_cos_axis;
  snapshot->imu_map_gyro_axis = imu_map.gyro_axis;
  snapshot->imu_map_sin_sign = imu_map.accel_sin_sign;
  snapshot->imu_map_cos_sign = imu_map.accel_cos_sign;
  snapshot->imu_map_gyro_sign = imu_map.gyro_sign;
  snapshot->imu_read_errors = imu_read_errors;

  if (last_logger_snapshot_us == 0U ||
      (snapshot->t_us - last_logger_snapshot_us) >= kLoggerSnapshotPeriodUs) {
    cached_logger_status = runtime_logger.status();
    last_logger_snapshot_us = snapshot->t_us;
  }
  snapshot->log_state = static_cast<std::uint8_t>(cached_logger_status.state);
  snapshot->log_partition_bytes = cached_logger_status.partition_bytes;
  snapshot->log_prepared_bytes = cached_logger_status.prepared_bytes;
  snapshot->log_max_records = cached_logger_status.max_records;
  snapshot->log_buffered_bytes = cached_logger_status.buffered_bytes;
  snapshot->log_records_written = cached_logger_status.records_written;
  snapshot->log_dropped_records = cached_logger_status.dropped_records;
  snapshot->log_logical_bytes = cached_logger_status.logical_bytes;
  snapshot->log_flash_writes_allowed = cached_logger_status.flash_writes_allowed;
  snapshot->log_critical_window = log_critical_window;
  snapshot->log_dump_active = binary_dump_active;
}

bool readEncoderDiagnosticStatus(EncoderDiagnosticStatus* const status) {
  if (status == nullptr) {
    return false;
  }

  static triwhirl::drivers::As5600Status cached{};
  static bool cached_valid = false;

  triwhirl::drivers::As5600Status current{};
  const bool ok = state::encoder.readStatus(&current);
  if (ok) {
    cached = current;
    cached_valid = true;
  }

  const triwhirl::drivers::As5600Status& value = ok ? current : cached;
  status->status_ok = ok;
  status->raw = cached_valid ? value.raw : 0U;
  status->magnet_detected = cached_valid && value.magnet_detected;
  status->magnet_too_weak = cached_valid && value.magnet_too_weak;
  status->magnet_too_strong = cached_valid && value.magnet_too_strong;
  return ok;
}

}  // namespace triwhirl::runtime
