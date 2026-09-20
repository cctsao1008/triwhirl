// Realtime control runtime for the established supervisor/swing path.
//
// Core 1 owns the 1 kHz control iteration. AS5600 acquisition is delegated to
// a dedicated Core-0 worker through runtime_encoder_acquisition; Core 1 performs
// the blocking MPU6050 transaction and attitude update in parallel. UART0 is a
// wired development/service ingress and BLE commands arrive through NimBLE GATT;
// both are parsed in the Core-0 supervisor domain before typed mutations enter
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
  triwhirl::runtime::publishRuntimeSnapshot(snapshot);
}

bool typedCommandAllowedDuringSwing(
    const triwhirl::runtime::RuntimeCommandType type) {
  if (!swing_id_runner.active()) {
    return true;
  }
  return type == triwhirl::runtime::RuntimeCommandType::kSwingAbort ||
         type == triwhirl::runtime::RuntimeCommandType::kTelemetryOff;
}

void executeRuntimeCommand(const triwhirl::runtime::RuntimeCommand& command) {
  if (!typedCommandAllowedDuringSwing(command.type)) {
    consoleWrite(
        "ERR swing experiment owns realtime actuation; use 'swing abort' first\r\n");
    return;
  }

  switch (command.type) {
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
    case triwhirl::runtime::RuntimeCommandType::kTimingReset:
      resetTimingStats();
      consoleWrite("OK timing reset\r\n");
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
  } else if (event.type == triwhirl::runtime::SupervisorInputEventType::kCommand) {
    handleSupervisorCommand(event.line);
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

  if (!triwhirl::runtime::initSupervisorIo(supervisorWrite, nullptr, 0,
                                            kSupervisorTaskPriority)) {
    safety_latch.trip(SafetyFault::kStartup);
    stopMotor();
    consoleWrite("FATAL fault=startup supervisor I/O task creation failed\r\n");
    vTaskDelete(nullptr);
    return;
  }

  publishSupervisorSnapshot(static_cast<std::uint32_t>(esp_timer_get_time()));

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
