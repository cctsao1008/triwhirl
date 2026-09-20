// Realtime control runtime for the established supervisor/swing path.
//
// Core 1 owns the 1 kHz control iteration. AS5600 acquisition is delegated to
// a dedicated Core-0 worker through runtime_encoder_acquisition; Core 1 performs
// the blocking MPU6050 transaction and attitude update in parallel. UART0 is a
// wired development/service ingress and BLE commands arrive through NimBLE GATT;
// both are parsed in the Core-0 supervisor domain before typed commands enter
// this domain through a bounded command mailbox.

#include <cstddef>
#include <cstdint>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "runtime_control.hpp"
#include "runtime_encoder_acquisition.hpp"
#include "runtime_release.hpp"
#include "runtime_snapshot.hpp"
#include "runtime_supervisor_io.hpp"

// runtime_main.cpp still owns the supervisor/swing runtime in this
// behavior-preserving refactor slice. Task selection, I2C affinity, encoder
// acquisition, supervisor transport, snapshots, and the GPTimer scheduler are
// explicit interfaces; the remaining composition debt is the source inclusion
// itself.
#include "runtime_main.cpp"

static_assert(triwhirl::runtime::kRealtimeReleasePeriodUs == kControlPeriodUs,
              "GPTimer release period must match control period");

namespace {

struct LocalTimingStats {
  std::uint64_t count = 0U;
  std::uint64_t total_us = 0U;
  std::uint32_t min_us = 0U;
  std::uint32_t max_us = 0U;
};

struct ControlPeriodHistogram {
  std::uint64_t lt_900 = 0U;
  std::uint64_t us_900_949 = 0U;
  std::uint64_t us_950_999 = 0U;
  std::uint64_t us_1000_1049 = 0U;
  std::uint64_t us_1050_1099 = 0U;
  std::uint64_t us_1100_1249 = 0U;
  std::uint64_t us_1250_1499 = 0U;
  std::uint64_t ge_1500 = 0U;
};

constexpr std::uint32_t kEncoderJoinBudgetUs = 100U;
constexpr std::uint32_t kEncoderConsecutiveMissLimit = 2U;
constexpr unsigned kSupervisorTaskPriority = 2U;

std::uint64_t encoder_acq_completions = 0U;
std::uint32_t encoder_acq_consecutive_misses = 0U;
std::uint32_t encoder_acq_max_consecutive_misses = 0U;

LocalTimingStats attitude_math_timing{};
ControlPeriodHistogram control_period_histogram{};
std::int64_t control_previous_start_us = 0;
bool control_profile_active = false;

void resetControlProfileStats() {
  triwhirl::runtime::resetEncoderAcquisitionStats();
  encoder_acq_completions = 0U;
  encoder_acq_consecutive_misses = 0U;
  encoder_acq_max_consecutive_misses = 0U;
  attitude_math_timing = {};
  control_period_histogram = {};
  control_previous_start_us = 0;
}

void recordLocalTiming(LocalTimingStats* const stats,
                       const std::int64_t begin_us,
                       const std::int64_t end_us) {
  if (stats == nullptr || end_us < begin_us) {
    return;
  }
  const std::uint32_t elapsed = static_cast<std::uint32_t>(end_us - begin_us);
  ++stats->count;
  stats->total_us += elapsed;
  if (stats->count == 1U || elapsed < stats->min_us) {
    stats->min_us = elapsed;
  }
  if (elapsed > stats->max_us) {
    stats->max_us = elapsed;
  }
}

void recordControlPeriod(const std::int64_t start_us) {
  if (control_previous_start_us == 0 || start_us <= control_previous_start_us) {
    control_previous_start_us = start_us;
    return;
  }
  const std::uint32_t period_us =
      static_cast<std::uint32_t>(start_us - control_previous_start_us);
  control_previous_start_us = start_us;
  if (period_us < 900U) {
    ++control_period_histogram.lt_900;
  } else if (period_us < 950U) {
    ++control_period_histogram.us_900_949;
  } else if (period_us < 1000U) {
    ++control_period_histogram.us_950_999;
  } else if (period_us < 1050U) {
    ++control_period_histogram.us_1000_1049;
  } else if (period_us < 1100U) {
    ++control_period_histogram.us_1050_1099;
  } else if (period_us < 1250U) {
    ++control_period_histogram.us_1100_1249;
  } else if (period_us < 1500U) {
    ++control_period_histogram.us_1250_1499;
  } else {
    ++control_period_histogram.ge_1500;
  }
}

void printControlProfileSummary() {
  const auto encoder_stats = triwhirl::runtime::encoderAcquisitionStats();
  const double attitude_mean_us =
      attitude_math_timing.count > 0U
          ? static_cast<double>(attitude_math_timing.total_us) /
                static_cast<double>(attitude_math_timing.count)
          : 0.0;
  consolePrintf(
      "parallel_profile,requests=%llu,completions=%llu,dispatch_failures=%llu,read_failures=%llu,stale_results=%llu,join_timeouts=%llu,max_consecutive_misses=%lu,attitude_count=%llu,attitude_mean_us=%.3f,attitude_min_us=%lu,attitude_max_us=%lu,period_lt900=%llu,period_900_949=%llu,period_950_999=%llu,period_1000_1049=%llu,period_1050_1099=%llu,period_1100_1249=%llu,period_1250_1499=%llu,period_ge1500=%llu\r\n",
      static_cast<unsigned long long>(encoder_stats.requests),
      static_cast<unsigned long long>(encoder_acq_completions),
      static_cast<unsigned long long>(encoder_stats.dispatch_failures),
      static_cast<unsigned long long>(encoder_stats.read_failures),
      static_cast<unsigned long long>(encoder_stats.stale_results),
      static_cast<unsigned long long>(encoder_stats.join_timeouts),
      static_cast<unsigned long>(encoder_acq_max_consecutive_misses),
      static_cast<unsigned long long>(attitude_math_timing.count),
      attitude_mean_us,
      static_cast<unsigned long>(attitude_math_timing.min_us),
      static_cast<unsigned long>(attitude_math_timing.max_us),
      static_cast<unsigned long long>(control_period_histogram.lt_900),
      static_cast<unsigned long long>(control_period_histogram.us_900_949),
      static_cast<unsigned long long>(control_period_histogram.us_950_999),
      static_cast<unsigned long long>(control_period_histogram.us_1000_1049),
      static_cast<unsigned long long>(control_period_histogram.us_1050_1099),
      static_cast<unsigned long long>(control_period_histogram.us_1100_1249),
      static_cast<unsigned long long>(control_period_histogram.us_1250_1499),
      static_cast<unsigned long long>(control_period_histogram.ge_1500));
}

void noteEncoderMiss() {
  ++encoder_read_errors;
  ++encoder_acq_consecutive_misses;
  if (encoder_acq_consecutive_misses > encoder_acq_max_consecutive_misses) {
    encoder_acq_max_consecutive_misses = encoder_acq_consecutive_misses;
  }

  if (encoder_acq_consecutive_misses >= kEncoderConsecutiveMissLimit) {
    encoder_sample_valid = false;
  }
}

bool readEncoderRaw(void*, std::uint16_t* const raw_count) {
  return encoder.readRawAngle(raw_count);
}

bool commitEncoderResult(
    const triwhirl::runtime::EncoderAcquisitionResult& result,
    const std::uint32_t sample_time_us) {
  if (!result.ok) {
    noteEncoderMiss();
    return false;
  }

  wheel_state = wheel_kinematics.update(result.raw_count, sample_time_us);
  encoder_sample_valid = true;
  encoder_acq_consecutive_misses = 0U;
  ++encoder_acq_completions;
  return true;
}

bool collectAndCommitEncoder(const std::uint32_t expected_sequence,
                             const std::uint32_t sample_time_us) {
  triwhirl::runtime::EncoderAcquisitionResult result{};
  if (!triwhirl::runtime::collectEncoderAcquisition(
          expected_sequence, kEncoderJoinBudgetUs, &result)) {
    noteEncoderMiss();
    return false;
  }
  return commitEncoderResult(result, sample_time_us);
}

void supervisorWrite(void*, const char* const data, const std::size_t length) {
  consoleWriteBytes(data, length);
}

void publishSupervisorSnapshot(const std::uint32_t now_us) {
  triwhirl::runtime::RuntimeSnapshot snapshot{};
  snapshot.t_us = now_us;
  snapshot.attitude_initialized = attitude_initialized;
  snapshot.attitude_valid = attitude_state.valid;
  snapshot.attitude_angle_rad = attitude_state.angle_rad;
  snapshot.attitude_rate_rad_s = attitude_state.rate_rad_s;
  snapshot.attitude_residual_bias_rad_s = attitude_state.gyro_bias_rad_s;
  snapshot.attitude_innovation = attitude_state.gravity_innovation;
  snapshot.attitude_accel_weight = attitude_state.accel_weight;
  snapshot.wheel_rate_rad_s = wheel_state.velocity_rad_s;
  snapshot.safety_faulted = safety_latch.faulted();
  snapshot.safety_fault_mask = safety_latch.mask();
  snapshot.safety_first_fault =
      static_cast<std::uint32_t>(safety_latch.firstFault());
  snapshot.telemetry_enabled = telemetry_enabled;
  snapshot.timing_target_us = kControlPeriodUs;
  snapshot.timing_hard_period_us = kHardControlPeriodUs;
  snapshot.timing_iterations = timing_stats.iterations;
  snapshot.timing_last_exec_us = timing_stats.last_exec_us;
  snapshot.timing_max_exec_us = timing_stats.max_exec_us;
  snapshot.timing_min_period_us =
      timing_stats.iterations > 1U ? timing_stats.min_period_us : 0U;
  snapshot.timing_max_period_us = timing_stats.max_period_us;
  snapshot.timing_overruns = timing_stats.overruns;
  snapshot.timing_late_periods = timing_stats.late_periods;
  snapshot.uart_tx_drop_bytes = console_tx_dropped_bytes;
  triwhirl::runtime::publishRuntimeSnapshot(snapshot);
}

bool typedCommandAllowedDuringSwing(
    const triwhirl::runtime::RuntimeCommandType type) {
  if (!swing_id_runner.active()) {
    return true;
  }
  return type == triwhirl::runtime::RuntimeCommandType::kStatus ||
         type == triwhirl::runtime::RuntimeCommandType::kImuStatus ||
         type == triwhirl::runtime::RuntimeCommandType::kLogStatus ||
         type == triwhirl::runtime::RuntimeCommandType::kSwingStatus ||
         type == triwhirl::runtime::RuntimeCommandType::kTimingProfileStatus ||
         type == triwhirl::runtime::RuntimeCommandType::kSwingAbort ||
         type == triwhirl::runtime::RuntimeCommandType::kSwingStart ||
         type == triwhirl::runtime::RuntimeCommandType::kSwingConfig ||
         type == triwhirl::runtime::RuntimeCommandType::kTimingProfileOn ||
         type == triwhirl::runtime::RuntimeCommandType::kTimingProfileOff ||
         type == triwhirl::runtime::RuntimeCommandType::kTimingProfileReset ||
         type == triwhirl::runtime::RuntimeCommandType::kTelemetryOff;
}

void executeRuntimeCommand(const triwhirl::runtime::RuntimeCommand& command) {
  if (!typedCommandAllowedDuringSwing(command.type)) {
    consoleWrite(
        "ERR swing experiment owns realtime actuation; use 'swing abort' first\r\n");
    return;
  }

  switch (command.type) {
    case triwhirl::runtime::RuntimeCommandType::kStatus:
    case triwhirl::runtime::RuntimeCommandType::kMotorStatus:
      printStatus();
      return;
    case triwhirl::runtime::RuntimeCommandType::kImuStatus:
      printImuStatus();
      return;
    case triwhirl::runtime::RuntimeCommandType::kLogStatus:
      printLogStatus();
      return;
    case triwhirl::runtime::RuntimeCommandType::kSwingStatus:
      printSwingStatus();
      return;
    case triwhirl::runtime::RuntimeCommandType::kTimingProfileStatus:
      printRuntimeTimingProfile();
      return;
    case triwhirl::runtime::RuntimeCommandType::kMotorStop:
      stopMotor();
      consoleWrite("OK motor stop\r\n");
      return;
    case triwhirl::runtime::RuntimeCommandType::kStop:
      stopMotor();
      consoleWrite("OK stop\r\n");
      return;
    case triwhirl::runtime::RuntimeCommandType::kSwingAbort:
      if (!swing_id_runner.active()) {
        consoleWrite("OK swing already inactive\r\n");
        return;
      }
      consoleWrite("OK swing abort\r\n");
      finishSwingRun(
          swing_id_runner.abort(SwingIdStopReason::kExternalAbort));
      return;
    case triwhirl::runtime::RuntimeCommandType::kSwingStart: {
      if (swing_id_runner.active()) {
        consoleWrite("ERR swing already active\r\n");
        return;
      }
      if (motorActive()) {
        consoleWrite("ERR swing start requires motor stopped\r\n");
        return;
      }
      if (!motor_config_valid || !encoder_sample_valid ||
          !wheel_state.velocity_valid || !imu_sample_valid || !gyro_bias_valid ||
          !attitude_state.valid || safety_latch.faulted()) {
        consoleWrite(
            "ERR swing start requires motor config, encoder/wheel, calibrated IMU, valid attitude, and clear safety\r\n");
        return;
      }
      const LoggerStatus log_status = runtime_logger.status();
      const std::uint32_t needed_records =
          (swing_id_runner.config().max_duration_us +
           triwhirl::log::kTwLogSamplePeriodUs - 1U) /
          triwhirl::log::kTwLogSamplePeriodUs;
      if (log_status.state != triwhirl::log::LoggerState::kRecording ||
          log_status.max_records < needed_records) {
        consoleWrite(
            "ERR swing start requires active TWLG recording with capacity for max duration\r\n");
        return;
      }
      const std::uint32_t now_us =
          static_cast<std::uint32_t>(esp_timer_get_time());
      if (!swing_id_runner.start(currentSwingInput(now_us))) {
        consoleWrite("ERR swing start rejected\r\n");
        return;
      }
      setSwingCriticalWindow(false);
      vq_command_v = clampFinite(swing_id_runner.output().desired_vq_v,
                                 -kMotorVectorLimitV, kMotorVectorLimitV);
      motor_mode = MotorMode::kFoc;
      consoleWrite("OK swing start\r\n");
      queueSwingEvent(swing_id_runner.output());
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kSwingConfig: {
      const auto& payload = command.payload.swing_config;
      SwingIdConfig config{};
      config.target_captures = payload.target_captures;
      config.pump_v_low = payload.pump_v_low;
      config.pump_v_high = payload.pump_v_high;
      config.capture_deg = payload.capture_deg;
      config.probe_exit_deg = payload.probe_exit_deg;
      config.rearm_deg = payload.rearm_deg;
      config.probe_duration_us = payload.probe_duration_us;
      config.rate_switch_rad_s = payload.rate_switch_rad_s;
      config.pump_polarity = payload.pump_polarity;
      config.vertex_a_deg = payload.vertex_a_deg;
      config.max_duration_us = payload.max_duration_us;
      if (config.pump_v_high > kMotorVectorLimitV ||
          config.pump_v_low > kMotorVectorLimitV ||
          !swing_id_runner.configure(config)) {
        consoleWrite(
            "ERR usage: swing config <captures> <pump_low_v> <pump_high_v> <capture_deg> <exit_deg> <rearm_deg> <probe_ms> <rate_switch_rad_s> <polarity> <vertex_a_deg> <max_s>\r\n");
        return;
      }
      consoleWrite("OK swing config\r\n");
      printSwingStatus();
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kTimingReset:
      resetTimingStats();
      consoleWrite("OK timing reset\r\n");
      return;
    case triwhirl::runtime::RuntimeCommandType::kTimingProfileOn:
      runtime_timing_profile.enabled = false;
      encoder.setTimingProfileEnabled(false);
      imu.setTimingProfileEnabled(false);
      resetRuntimeTimingProfile();
      encoder.setTimingProfileEnabled(true);
      imu.setTimingProfileEnabled(true);
      runtime_timing_profile.enabled = true;
      consoleWrite("OK timing profile on\r\n");
      return;
    case triwhirl::runtime::RuntimeCommandType::kTimingProfileOff:
      runtime_timing_profile.enabled = false;
      encoder.setTimingProfileEnabled(false);
      imu.setTimingProfileEnabled(false);
      consoleWrite("OK timing profile off\r\n");
      printRuntimeTimingProfile();
      return;
    case triwhirl::runtime::RuntimeCommandType::kTimingProfileReset:
      resetRuntimeTimingProfile();
      consoleWrite("OK timing profile reset\r\n");
      return;
    case triwhirl::runtime::RuntimeCommandType::kFaultClear:
      stopMotor();
      if (!safety_latch.faulted()) {
        consoleWrite("OK fault already clear\r\n");
        return;
      }
      if (!faultClearReady()) {
        consoleWrite("ERR fault clear rejected; fault cause is still present\r\n");
        return;
      }
      safety_latch.clear();
      consoleWrite("OK fault clear\r\n");
      return;
    case triwhirl::runtime::RuntimeCommandType::kTelemetryOn:
      telemetry_enabled = true;
      last_telemetry_us = static_cast<std::uint32_t>(esp_timer_get_time());
      consoleWrite("OK telemetry on\r\n");
      return;
    case triwhirl::runtime::RuntimeCommandType::kTelemetryOff:
      telemetry_enabled = false;
      consoleWrite("OK telemetry off\r\n");
      return;
    case triwhirl::runtime::RuntimeCommandType::kMotorVq: {
      const float requested_vq = clampFinite(
          command.payload.motor_vq.volts, -kMotorVectorLimitV,
          kMotorVectorLimitV);
      if (std::fabs(requested_vq) < 1.0e-4F) {
        stopMotor();
        consoleWrite("OK motor stop\r\n");
        return;
      }
      if (!motorStartAllowed()) {
        return;
      }
      if (!motor_config_valid) {
        consoleWrite("ERR motor is not calibrated/configured\r\n");
        return;
      }
      vq_command_v = requested_vq;
      motor_mode = MotorMode::kFoc;
      consolePrintf("OK motor FOC vq_v=%.6f\r\n", vq_command_v);
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kMotorConfig: {
      MotorElectricalConfig config{};
      config.pole_pairs = command.payload.motor_config.pole_pairs;
      config.sensor_direction = command.payload.motor_config.sensor_direction;
      config.electrical_offset_rad = triwhirl::wrapElectricalAngle(
          command.payload.motor_config.electrical_offset_rad);
      if (!triwhirl::validMotorElectricalConfig(config)) {
        consoleWrite("ERR invalid motor config\r\n");
        return;
      }
      stopMotor();
      motor_config = config;
      motor_config_valid = true;
      consolePrintf(
          "OK motor config pole_pairs=%d sensor_dir=%d offset_rad=%.6f\r\n",
          motor_config.pole_pairs, motor_config.sensor_direction,
          motor_config.electrical_offset_rad);
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kMotorCalibrate:
      if (!motorStartAllowed()) {
        return;
      }
      stopMotor();
      if (!encoder_sample_valid) {
        consoleWrite("ERR motor calibrate: encoder read unavailable\r\n");
        return;
      }
      calibration.amplitude_v = clampFinite(
          command.payload.motor_calibrate.amplitude_v, 0.1F,
          kMotorVectorLimitV);
      calibration.electrical_hz = clampFinite(
          command.payload.motor_calibrate.electrical_hz, 0.1F, 2.0F);
      calibration.electrical_turns = clampFinite(
          command.payload.motor_calibrate.turns, 1.0F, 12.0F);
      calibration.commanded_electrical_rad = 0.0F;
      calibration.start_mechanical_rad = wheel_state.unwrapped_angle_rad;
      calibration.stage = CalibrationStage::kAlign;
      calibration.stage_start_us =
          static_cast<std::uint32_t>(esp_timer_get_time());
      motor_mode = MotorMode::kCalibrating;
      vq_command_v = 0.0F;
      consolePrintf(
          "OK motor calibration started amp_v=%.3f e_hz=%.3f turns=%.3f\r\n",
          calibration.amplitude_v, calibration.electrical_hz,
          calibration.electrical_turns);
      return;
    case triwhirl::runtime::RuntimeCommandType::kField: {
      const float requested_hz = clampFinite(
          command.payload.field.electrical_hz, -kMaxElectricalHz,
          kMaxElectricalHz);
      const float requested_amplitude = clampFinite(
          command.payload.field.amplitude_v, 0.0F, kMotorVectorLimitV);
      if (requested_amplitude <= 0.0F || requested_hz == 0.0F) {
        stopMotor();
        consoleWrite("OK field stopped\r\n");
        return;
      }
      if (!motorStartAllowed()) {
        return;
      }
      stopMotor();
      open_loop_hz = requested_hz;
      open_loop_amplitude_v = requested_amplitude;
      open_loop_angle_rad = 0.0F;
      motor_mode = MotorMode::kOpenLoop;
      consolePrintf("OK field e_hz=%.6f amp_v=%.6f\r\n", open_loop_hz,
                    open_loop_amplitude_v);
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kAttitudeReset:
      if (command.payload.attitude_reset.use_accelerometer) {
        resetAttitudeFromAccel();
        consoleWrite("OK attitude reset from accelerometer\r\n");
        return;
      }
      if (!std::isfinite(command.payload.attitude_reset.angle_rad)) {
        consoleWrite("ERR invalid attitude angle\r\n");
        return;
      }
      attitude_estimator.reset(command.payload.attitude_reset.angle_rad, 0.0F);
      attitude_state = attitude_estimator.state();
      attitude_initialized = true;
      last_attitude_update_us = static_cast<std::uint32_t>(esp_timer_get_time());
      consolePrintf("OK attitude reset angle_rad=%.6f\r\n",
                    command.payload.attitude_reset.angle_rad);
      return;
    case triwhirl::runtime::RuntimeCommandType::kImuCalibrate:
      startGyroCalibration(command.payload.imu_calibrate.samples);
      return;
    case triwhirl::runtime::RuntimeCommandType::kImuMap: {
      ImuPlanarMap map{};
      map.accel_sin_axis = command.payload.imu_map.accel_sin_axis;
      map.accel_cos_axis = command.payload.imu_map.accel_cos_axis;
      map.gyro_axis = command.payload.imu_map.gyro_axis;
      map.accel_sin_sign = command.payload.imu_map.accel_sin_sign;
      map.accel_cos_sign = command.payload.imu_map.accel_cos_sign;
      map.gyro_sign = command.payload.imu_map.gyro_sign;
      if (!validImuMap(map)) {
        consoleWrite("ERR invalid imu map\r\n");
        return;
      }
      imu_map = map;
      resetAttitudeFromAccel();
      consolePrintf("OK imu map %d %d %d %d %d %d\r\n",
                    imu_map.accel_sin_axis, imu_map.accel_cos_axis,
                    imu_map.gyro_axis, imu_map.accel_sin_sign,
                    imu_map.accel_cos_sign, imu_map.gyro_sign);
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kLogPrepare: {
      const float seconds = command.payload.log_prepare.seconds;
      if (motorActive()) {
        consoleWrite("ERR log prepare requires motor stopped\r\n");
        return;
      }
      if (!std::isfinite(seconds) || seconds <= 0.0F) {
        consoleWrite("ERR log prepare seconds must be > 0\r\n");
        return;
      }
      const double records_d = std::ceil(
          static_cast<double>(seconds) * 1000000.0 /
          static_cast<double>(triwhirl::log::kTwLogSamplePeriodUs));
      if (records_d < 1.0 ||
          records_d > static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
        consoleWrite("ERR log prepare duration out of range\r\n");
        return;
      }
      const std::uint32_t records = static_cast<std::uint32_t>(records_d);
      if (!runtime_logger.prepare(records)) {
        consoleWrite("ERR log prepare rejected; check log status/capacity\r\n");
        return;
      }
      log_critical_window = false;
      consolePrintf(
          "OK log prepare records=%lu seconds=%.3f; erase in background\r\n",
          static_cast<unsigned long>(records), seconds);
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kLogStart:
      if (!runtime_logger.start()) {
        consoleWrite("ERR log start requires state=ready\r\n");
        return;
      }
      log_critical_window = false;
      consoleWrite("OK log start sample_us=1000 record_bytes=32\r\n");
      return;
    case triwhirl::runtime::RuntimeCommandType::kLogCriticalOn:
      log_critical_window = true;
      runtime_logger.setFlashWritesAllowed(false);
      consoleWrite("OK log critical on; flash programming paused\r\n");
      return;
    case triwhirl::runtime::RuntimeCommandType::kLogCriticalOff:
      log_critical_window = false;
      runtime_logger.setFlashWritesAllowed(true);
      consoleWrite("OK log critical off; flash programming resumed\r\n");
      return;
    case triwhirl::runtime::RuntimeCommandType::kLogStop:
      log_critical_window = false;
      runtime_logger.setFlashWritesAllowed(true);
      if (!runtime_logger.stop()) {
        consoleWrite("ERR log stop rejected; check log status\r\n");
        return;
      }
      consoleWrite("OK log stopping; SRAM is draining and header will finalize\r\n");
      return;
    case triwhirl::runtime::RuntimeCommandType::kLogDump:
      if (binary_dump_active || log_dump_task != nullptr) {
        consoleWrite("ERR log dump already active\r\n");
        return;
      }
      if (!runtime_logger.complete()) {
        consoleWrite("ERR log dump requires state=complete\r\n");
        return;
      }
      if (motorActive()) {
        consoleWrite("ERR log dump requires motor stopped\r\n");
        return;
      }
      if (!triwhirl::ble::connected() || !triwhirl::ble::subscribed()) {
        consoleWrite("ERR log dump requires BLE notify subscription\r\n");
        return;
      }
      telemetry_enabled = false;
      binary_dump_active = true;
      if (xTaskCreatePinnedToCore(logDumpTask, "triwhirl_log_dump", 4096, nullptr,
                                  1, &log_dump_task, 0) != pdPASS) {
        binary_dump_active = false;
        log_dump_task = nullptr;
        consoleWrite("ERR log dump task creation failed\r\n");
      }
      return;
    case triwhirl::runtime::RuntimeCommandType::kNone:
      return;
  }
}

void processOneSupervisorInput() {
  triwhirl::runtime::SupervisorInputEvent event{};
  if (!triwhirl::runtime::tryReceiveSupervisorInput(&event)) {
    return;
  }

  if (event.type == triwhirl::runtime::SupervisorInputEventType::kRuntimeCommand) {
    executeRuntimeCommand(event.runtime_command);
  }

  if (!swing_id_runner.active()) {
    printPrompt();
  }
}

void realtimeControlTaskImpl(void*) {
  if (!triwhirl::runtime::initEncoderAcquisition(
          readEncoderRaw, nullptr, 0,
          static_cast<unsigned>(configMAX_PRIORITIES - 1))) {
    safety_latch.trip(SafetyFault::kStartup);
    stopMotor();
    consoleWrite("FATAL fault=startup parallel encoder task creation failed\r\n");
    vTaskDelete(nullptr);
    return;
  }

  if (!triwhirl::runtime::initRuntimeSnapshotChannel()) {
    safety_latch.trip(SafetyFault::kStartup);
    stopMotor();
    consoleWrite("FATAL fault=startup runtime snapshot channel creation failed\r\n");
    vTaskDelete(nullptr);
    return;
  }

  publishSupervisorSnapshot(static_cast<std::uint32_t>(esp_timer_get_time()));

  if (!triwhirl::runtime::initSupervisorIo(supervisorWrite, nullptr, 0,
                                            kSupervisorTaskPriority)) {
    safety_latch.trip(SafetyFault::kStartup);
    stopMotor();
    consoleWrite("FATAL fault=startup supervisor I/O task creation failed\r\n");
    vTaskDelete(nullptr);
    return;
  }

  TickType_t last_wake = xTaskGetTickCount();
  while (true) {
    const std::int64_t start_us = esp_timer_get_time();
    const std::uint32_t loop_us = static_cast<std::uint32_t>(start_us);
    const bool profile = runtime_timing_profile.enabled;

    if (profile && !control_profile_active) {
      resetControlProfileStats();
      control_profile_active = true;
    } else if (!profile && control_profile_active) {
      control_profile_active = false;
      printControlProfileSummary();
    }
    if (profile) {
      recordControlPeriod(start_us);
    }

    std::uint32_t encoder_sequence = 0U;
    const bool encoder_dispatched =
        triwhirl::runtime::dispatchEncoderAcquisition(&encoder_sequence);

    const std::int64_t imu_begin_us = esp_timer_get_time();
    const bool imu_sampled = sampleImu();
    const std::int64_t attitude_begin_us = esp_timer_get_time();
    if (imu_sampled) {
      updateAttitude(loop_us);
    }
    const std::int64_t imu_end_us = esp_timer_get_time();
    if (profile) {
      recordRuntimeTimingStage(RuntimeTimingStage::kImuAttitude, imu_begin_us,
                               imu_end_us);
      if (imu_sampled) {
        recordLocalTiming(&attitude_math_timing, attitude_begin_us, imu_end_us);
      }
    }

    const std::int64_t encoder_join_begin_us = imu_end_us;
    if (encoder_dispatched) {
      collectAndCommitEncoder(encoder_sequence, loop_us);
    } else {
      noteEncoderMiss();
    }
    const std::int64_t encoder_join_end_us = esp_timer_get_time();
    if (profile) {
      recordRuntimeTimingStage(RuntimeTimingStage::kEncoder,
                               encoder_join_begin_us, encoder_join_end_us);
    }

    std::int64_t stage_us = encoder_join_end_us;

    evaluateSafety(start_us);
    updateSwingIdentification(loop_us);
    if (profile) {
      const std::int64_t now = esp_timer_get_time();
      recordRuntimeTimingStage(RuntimeTimingStage::kSafetySwing, stage_us, now);
      stage_us = now;
    }

    updateMotor(loop_us);
    if (profile) {
      const std::int64_t now = esp_timer_get_time();
      recordRuntimeTimingStage(RuntimeTimingStage::kMotor, stage_us, now);
      stage_us = now;
    }

    recordSwingRuntimeLog(loop_us);
    finalizeSwingLogIfPending();
    publishSupervisorSnapshot(loop_us);
    if (profile) {
      const std::int64_t now = esp_timer_get_time();
      recordRuntimeTimingStage(RuntimeTimingStage::kLog, stage_us, now);
      stage_us = now;
    }

    processOneSupervisorInput();
    finalizeSwingLogIfPending();
    if (profile) {
      const std::int64_t now = esp_timer_get_time();
      recordRuntimeTimingStage(RuntimeTimingStage::kConsole, stage_us, now);
      stage_us = now;
    }

    emitTelemetry(loop_us);
    const std::int64_t end_us = esp_timer_get_time();
    if (profile) {
      recordRuntimeTimingStage(RuntimeTimingStage::kTelemetry, stage_us, end_us);
      recordRuntimeTimingStage(RuntimeTimingStage::kLoop, start_us, end_us);
    }
    updateTimingStats(start_us, end_us);

    if (!triwhirl::runtime::waitForNextRealtimeRelease()) {
      // Preserve the previous scheduler fallback while this refactor remains
      // behavior-only. A later #32 safety slice will make GPTimer availability
      // an explicit Balance-mode admission condition.
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
  }
}

}  // namespace

void triwhirl::runtime::realtimeControlTask(void* opaque) {
  realtimeControlTaskImpl(opaque);
}
