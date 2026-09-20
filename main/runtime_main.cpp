// Explicit TriWhirl application startup and Core-1 realtime runtime.
// Shared bring-up state/helpers live in runtime_state.cpp; no source files are
// textually included into this translation unit.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "runtime_control.hpp"
#include "runtime_encoder_acquisition.hpp"
#include "runtime_platform.hpp"
#include "runtime_release.hpp"
#include "runtime_snapshot.hpp"
#include "runtime_state.hpp"
#include "runtime_supervisor_io.hpp"
#include "triwhirl/ble_transport.hpp"
#include "triwhirl/board.hpp"
#include "triwhirl/swing_id.hpp"

using namespace triwhirl::runtime::state;

static_assert(triwhirl::runtime::kRealtimeReleasePeriodUs == kControlPeriodUs,
              "GPTimer release period must match control period");

namespace {

using triwhirl::SwingIdConfig;
using triwhirl::SwingIdInput;
using triwhirl::SwingIdOutput;
using triwhirl::SwingIdRunner;
using triwhirl::SwingIdState;
using triwhirl::SwingIdStopReason;
using triwhirl::SwingIdVertex;

bool initRuntimeEncoderBus(i2c_master_bus_handle_t* bus) {
  i2c_master_bus_config_t config{};
  config.i2c_port = I2C_NUM_0;
  config.sda_io_num = static_cast<gpio_num_t>(triwhirl::board::kAs5600SdaGpio);
  config.scl_io_num = static_cast<gpio_num_t>(triwhirl::board::kAs5600SclGpio);
  config.clk_source = I2C_CLK_SRC_DEFAULT;
  config.glitch_ignore_cnt = 7;
  config.flags.enable_internal_pullup = true;
  return triwhirl::runtime::createI2cMasterBusOnCore(&config, bus, 0) == ESP_OK;
}

bool initRuntimeImuBus(i2c_master_bus_handle_t* bus) {
  i2c_master_bus_config_t config{};
  config.i2c_port = I2C_NUM_1;
  config.sda_io_num = static_cast<gpio_num_t>(triwhirl::board::kMpu6050SdaGpio);
  config.scl_io_num = static_cast<gpio_num_t>(triwhirl::board::kMpu6050SclGpio);
  config.clk_source = I2C_CLK_SRC_DEFAULT;
  config.glitch_ignore_cnt = 7;
  config.flags.enable_internal_pullup = true;
  return triwhirl::runtime::createI2cMasterBusOnCore(&config, bus, 1) == ESP_OK;
}

enum class RuntimeTimingStage : std::uint8_t {
  kEncoder = 0,
  kImuAttitude,
  kSafetySwing,
  kMotor,
  kLog,
  kConsole,
  kTelemetry,
  kLoop,
  kCount,
};

struct RuntimeTimingStageStats {
  std::uint64_t count = 0U;
  std::uint64_t total_us = 0U;
  std::uint32_t min_us = std::numeric_limits<std::uint32_t>::max();
  std::uint32_t max_us = 0U;
};

struct RuntimeTimingProfile {
  bool enabled = false;
  RuntimeTimingStageStats stages[static_cast<std::size_t>(RuntimeTimingStage::kCount)]{};
};

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

SwingIdRunner swing_id_runner{};
std::uint32_t swing_event_drops = 0U;
bool swing_log_finalize_pending = false;
RuntimeTimingProfile runtime_timing_profile{};

std::uint64_t encoder_acq_completions = 0U;
std::uint32_t encoder_acq_consecutive_misses = 0U;
std::uint32_t encoder_acq_max_consecutive_misses = 0U;
LocalTimingStats attitude_math_timing{};
ControlPeriodHistogram control_period_histogram{};
std::int64_t control_previous_start_us = 0;
bool control_profile_active = false;
bool command_reply_deferred = false;

void printSwingHelp();

void resetRuntimeTimingProfile() {
  const bool enabled = runtime_timing_profile.enabled;
  runtime_timing_profile = {};
  runtime_timing_profile.enabled = enabled;
  encoder.resetTimingProfile();
  imu.resetTimingProfile();
}

void recordRuntimeTimingStage(const RuntimeTimingStage stage,
                              const std::int64_t begin_us,
                              const std::int64_t end_us) {
  if (!runtime_timing_profile.enabled || end_us < begin_us ||
      stage == RuntimeTimingStage::kCount) {
    return;
  }
  const std::uint64_t elapsed64 = static_cast<std::uint64_t>(end_us - begin_us);
  const std::uint32_t elapsed =
      elapsed64 > std::numeric_limits<std::uint32_t>::max()
          ? std::numeric_limits<std::uint32_t>::max()
          : static_cast<std::uint32_t>(elapsed64);
  RuntimeTimingStageStats& stats =
      runtime_timing_profile.stages[static_cast<std::size_t>(stage)];
  ++stats.count;
  stats.total_us += elapsed;
  if (elapsed < stats.min_us) stats.min_us = elapsed;
  if (elapsed > stats.max_us) stats.max_us = elapsed;
}

bool promptAllowedNow() {
  return !swing_id_runner.active() && !telemetry_enabled && !binary_dump_active;
}

void publishReplyRecord(triwhirl::runtime::RuntimeReply reply,
                        const bool prompt_after) {
  reply.prompt_after = prompt_after;
  command_reply_deferred = true;
  triwhirl::runtime::publishRuntimeReply(reply);
}

void deferRuntimeReply(const triwhirl::runtime::RuntimeReplyCode code) {
  triwhirl::runtime::RuntimeReply reply{};
  reply.code = code;
  publishReplyRecord(reply, promptAllowedNow());
}

void deferRuntimeReply(const triwhirl::runtime::RuntimeReply& reply) {
  publishReplyRecord(reply, promptAllowedNow());
}

void publishTimingProfileStage(
    const triwhirl::runtime::RuntimeTimingProfileStageId stage,
    const std::uint64_t count, const std::uint64_t total_us,
    const std::uint32_t min_us, const std::uint32_t max_us) {
  triwhirl::runtime::RuntimeReply reply{};
  reply.code = triwhirl::runtime::RuntimeReplyCode::kTimingProfileStage;
  reply.value0 = static_cast<std::int32_t>(stage);
  reply.wide0 = count;
  reply.wide1 = total_us;
  reply.u32_0 = min_us;
  reply.u32_1 = max_us;
  publishReplyRecord(reply, false);
}

void publishRuntimeTimingProfile(const bool prompt_after) {
  const RuntimeTimingStageStats& loop =
      runtime_timing_profile.stages[static_cast<std::size_t>(RuntimeTimingStage::kLoop)];

  triwhirl::runtime::RuntimeReply header{};
  header.code = triwhirl::runtime::RuntimeReplyCode::kTimingProfileHeader;
  header.value0 = runtime_timing_profile.enabled ? 1 : 0;
  header.wide0 = loop.count;
  publishReplyRecord(header, false);

  for (std::size_t index = 0U;
       index < static_cast<std::size_t>(RuntimeTimingStage::kCount); ++index) {
    const RuntimeTimingStageStats& stats = runtime_timing_profile.stages[index];
    publishTimingProfileStage(
        static_cast<triwhirl::runtime::RuntimeTimingProfileStageId>(index),
        stats.count, stats.total_us, stats.min_us, stats.max_us);
  }

  const auto encoder_timing = encoder.timingProfile();
  publishTimingProfileStage(
      triwhirl::runtime::RuntimeTimingProfileStageId::kEncoderI2cRaw,
      encoder_timing.raw_reads, encoder_timing.raw_total_us,
      encoder_timing.raw_min_us, encoder_timing.raw_max_us);
  publishTimingProfileStage(
      triwhirl::runtime::RuntimeTimingProfileStageId::kEncoderI2cStatus,
      encoder_timing.status_reads, encoder_timing.status_total_us,
      encoder_timing.status_min_us, encoder_timing.status_max_us);

  const auto imu_timing = imu.timingProfile();
  publishTimingProfileStage(
      triwhirl::runtime::RuntimeTimingProfileStageId::kMpuI2c,
      imu_timing.sample_reads, imu_timing.transfer_total_us,
      imu_timing.transfer_min_us, imu_timing.transfer_max_us);
  publishTimingProfileStage(
      triwhirl::runtime::RuntimeTimingProfileStageId::kMpuDecode,
      imu_timing.sample_reads, imu_timing.decode_total_us,
      imu_timing.decode_min_us, imu_timing.decode_max_us);

  triwhirl::runtime::RuntimeReply end{};
  end.code = triwhirl::runtime::RuntimeReplyCode::kTimingProfileEnd;
  publishReplyRecord(end, prompt_after);
}

bool swingTerminal(const SwingIdState state) {
  return state == SwingIdState::kComplete || state == SwingIdState::kAborted;
}

triwhirl::runtime::RuntimeReply makeSwingStatusReply(
    const triwhirl::runtime::RuntimeReplyCode code) {
  const SwingIdOutput& output = swing_id_runner.output();
  const SwingIdConfig& config = swing_id_runner.config();
  triwhirl::runtime::RuntimeReply reply{};
  reply.code = code;
  reply.value0 = static_cast<std::int32_t>(output.state);
  reply.value1 = static_cast<std::int32_t>(output.stop_reason);
  reply.value2 = static_cast<std::int32_t>(output.capture_count);
  reply.value3 = static_cast<std::int32_t>(config.target_captures);
  reply.value4 = static_cast<std::int32_t>(output.half_cycle_index);
  reply.value5 = static_cast<std::int32_t>(output.vertex);
  reply.value6 = (output.pump_active ? 0x01 : 0) |
                 (output.probe_active ? 0x02 : 0) |
                 (output.critical_window ? 0x04 : 0);
  reply.value7 = static_cast<std::int32_t>(swing_event_drops);
  reply.value8 = config.pump_polarity;
  reply.u32_0 = config.probe_duration_us;
  reply.u32_1 = config.max_duration_us;
  reply.float0 = output.vertex_error_deg;
  reply.float1 = output.desired_vq_v;
  reply.float2 = config.pump_v_low;
  reply.float3 = config.pump_v_high;
  reply.float4 = config.capture_deg;
  reply.float5 = config.probe_exit_deg;
  reply.float6 = config.rearm_deg;
  reply.float7 = config.rate_switch_rad_s;
  reply.float8 = config.vertex_a_deg;
  return reply;
}

void queueSwingEvent(const SwingIdOutput& output) {
  if (!output.transition) return;
  triwhirl::runtime::RuntimeReply reply{};
  reply.code = triwhirl::runtime::RuntimeReplyCode::kSwingTransitionEvent;
  reply.value0 = static_cast<std::int32_t>(output.state);
  reply.value1 = static_cast<std::int32_t>(output.capture_count);
  reply.value2 = static_cast<std::int32_t>(swing_id_runner.config().target_captures);
  reply.value3 = static_cast<std::int32_t>(output.half_cycle_index);
  reply.value4 = static_cast<std::int32_t>(output.vertex);
  reply.value5 = static_cast<std::int32_t>(output.stop_reason);
  reply.u32_0 = safety_latch.mask();
  reply.float0 = output.vertex_error_deg;
  reply.float1 = output.desired_vq_v;
  if (!triwhirl::runtime::publishRuntimeReply(reply)) {
    ++swing_event_drops;
  }
}

void setSwingCriticalWindow(const bool critical) {
  if (log_critical_window == critical) return;
  log_critical_window = critical;
  runtime_logger.setFlashWritesAllowed(!critical);
}

void finishSwingRun(const SwingIdOutput& output) {
  setSwingCriticalWindow(false);
  stopMotor();
  swing_log_finalize_pending = true;
  queueSwingEvent(output);
}

void applySwingOutput(const SwingIdOutput& output) {
  setSwingCriticalWindow(output.critical_window);
  if (swing_id_runner.active()) {
    vq_command_v = clampFinite(output.desired_vq_v, -kMotorVectorLimitV,
                               kMotorVectorLimitV);
    motor_mode = MotorMode::kFoc;
  } else if (swingTerminal(output.state)) {
    finishSwingRun(output);
    return;
  }
  queueSwingEvent(output);
}

SwingIdInput currentSwingInput(const std::uint32_t now_us) {
  SwingIdInput input{};
  input.now_us = now_us;
  input.theta_rad = attitude_state.angle_rad;
  input.theta_rate_rad_s = attitude_state.rate_rad_s;
  input.attitude_valid = attitude_state.valid && imu_sample_valid && gyro_bias_valid;
  input.safety_faulted = safety_latch.faulted();
  return input;
}

void updateSwingIdentification(const std::uint32_t now_us) {
  if (!swing_id_runner.active()) return;
  applySwingOutput(swing_id_runner.update(currentSwingInput(now_us)));
}

void finalizeSwingLogIfPending() {
  if (!swing_log_finalize_pending) return;
  setSwingCriticalWindow(false);
  runtime_logger.stop();
  swing_log_finalize_pending = false;
}

// Startup-only formatter. Runtime command/status egress uses RuntimeReply and is
// formatted by the Core-0 supervisor.
void printSwingStatus() {
  const SwingIdOutput& output = swing_id_runner.output();
  const SwingIdConfig& config = swing_id_runner.config();
  consolePrintf(
      "swing,state=%s,reason=%s,captures=%lu,target=%lu,half_cycle=%lu,vertex=%s,error_deg=%.3f,vq_v=%.3f,pump=%d,probe=%d,critical=%d,event_drops=%lu,pump_low=%.3f,pump_high=%.3f,capture_deg=%.3f,exit_deg=%.3f,rearm_deg=%.3f,probe_ms=%.3f,rate_switch=%.6f,polarity=%d,vertex_a_deg=%.3f,max_s=%.3f\r\n",
      triwhirl::swingIdStateName(output.state),
      triwhirl::swingIdStopReasonName(output.stop_reason),
      static_cast<unsigned long>(output.capture_count),
      static_cast<unsigned long>(config.target_captures),
      static_cast<unsigned long>(output.half_cycle_index),
      triwhirl::swingIdVertexName(output.vertex), output.vertex_error_deg,
      output.desired_vq_v, output.pump_active ? 1 : 0,
      output.probe_active ? 1 : 0, output.critical_window ? 1 : 0,
      static_cast<unsigned long>(swing_event_drops), config.pump_v_low,
      config.pump_v_high, config.capture_deg, config.probe_exit_deg,
      config.rearm_deg, static_cast<float>(config.probe_duration_us) * 1.0e-3F,
      config.rate_switch_rad_s, config.pump_polarity, config.vertex_a_deg,
      static_cast<float>(config.max_duration_us) * 1.0e-6F);
}

std::uint16_t swingRuntimeLogFlags() {
  std::uint16_t flags = runtimeLogFlags();
  const SwingIdOutput& output = swing_id_runner.output();
  if (!swing_id_runner.active()) return flags;
  if (output.pump_active) flags |= triwhirl::log::kRecordPumpActive;
  if (output.probe_active) {
    flags |= triwhirl::log::kRecordProbeActive;
    switch (output.vertex) {
      case SwingIdVertex::kA: flags |= triwhirl::log::kRecordVertexA; break;
      case SwingIdVertex::kB: flags |= triwhirl::log::kRecordVertexB; break;
      case SwingIdVertex::kC: flags |= triwhirl::log::kRecordVertexC; break;
      case SwingIdVertex::kNone: break;
    }
  }
  return flags;
}

void recordSwingRuntimeLog(const std::uint32_t now_us) {
  RuntimeLogRecord record{};
  record.t_us = now_us;
  record.theta_rad = attitude_state.angle_rad;
  record.theta_rate_rad_s = attitude_state.rate_rad_s;
  record.wheel_rate_rad_s = wheel_state.velocity_rad_s;
  record.vq_v = vq_command_v;
  record.accel_weight = attitude_state.accel_weight;
  record.fault_mask = safety_latch.mask();
  record.flags = swingRuntimeLogFlags();
  record.raw_count = wheel_state.raw_count;
  runtime_logger.record(record);
}

void printSwingHelp() {
  consoleWrite("  swing status\r\n");
  consoleWrite("  swing config <captures> <pump_low_v> <pump_high_v> <capture_deg> <exit_deg> <rearm_deg> <probe_ms> <rate_switch_rad_s> <polarity> <vertex_a_deg> <max_s>\r\n");
  consoleWrite("  swing start\r\n");
  consoleWrite("  swing abort\r\n");
  consoleWrite("  timing profile <status|on|off|reset>\r\n");
}

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
  if (stats == nullptr || end_us < begin_us) return;
  const std::uint32_t elapsed = static_cast<std::uint32_t>(end_us - begin_us);
  ++stats->count;
  stats->total_us += elapsed;
  if (stats->count == 1U || elapsed < stats->min_us) stats->min_us = elapsed;
  if (elapsed > stats->max_us) stats->max_us = elapsed;
}

void recordControlPeriod(const std::int64_t start_us) {
  if (control_previous_start_us == 0 || start_us <= control_previous_start_us) {
    control_previous_start_us = start_us;
    return;
  }
  const std::uint32_t period_us =
      static_cast<std::uint32_t>(start_us - control_previous_start_us);
  control_previous_start_us = start_us;
  if (period_us < 900U) ++control_period_histogram.lt_900;
  else if (period_us < 950U) ++control_period_histogram.us_900_949;
  else if (period_us < 1000U) ++control_period_histogram.us_950_999;
  else if (period_us < 1050U) ++control_period_histogram.us_1000_1049;
  else if (period_us < 1100U) ++control_period_histogram.us_1050_1099;
  else if (period_us < 1250U) ++control_period_histogram.us_1100_1249;
  else if (period_us < 1500U) ++control_period_histogram.us_1250_1499;
  else ++control_period_histogram.ge_1500;
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
  snapshot.swing_active = swing_id_runner.active();
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

bool deferMotorStartFailure() {
  const MotorStartFailure failure = motorStartFailure();
  if (failure == MotorStartFailure::kNone) {
    return false;
  }
  triwhirl::runtime::RuntimeReply reply{};
  switch (failure) {
    case MotorStartFailure::kNone:
      return false;
    case MotorStartFailure::kSafetyFault:
      reply.code = triwhirl::runtime::RuntimeReplyCode::kSafetyFaultLatched;
      reply.value0 = static_cast<std::int32_t>(safety_latch.firstFault());
      reply.u32_0 = safety_latch.mask();
      break;
    case MotorStartFailure::kEncoderUnavailable:
      reply.code = triwhirl::runtime::RuntimeReplyCode::kEncoderUnavailable;
      break;
    case MotorStartFailure::kInvalidNumeric:
      reply.code = triwhirl::runtime::RuntimeReplyCode::kInvalidRuntimeNumeric;
      break;
  }
  deferRuntimeReply(reply);
  return true;
}

bool typedCommandAllowedDuringSwing(
    const triwhirl::runtime::RuntimeCommandType type) {
  if (!swing_id_runner.active()) return true;
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
    deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kSwingOwnsRealtime);
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
      deferRuntimeReply(makeSwingStatusReply(
          triwhirl::runtime::RuntimeReplyCode::kSwingStatus));
      return;
    case triwhirl::runtime::RuntimeCommandType::kTimingProfileStatus:
      publishRuntimeTimingProfile(promptAllowedNow());
      return;
    case triwhirl::runtime::RuntimeCommandType::kMotorStop:
      stopMotor();
      deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kMotorStopOk);
      return;
    case triwhirl::runtime::RuntimeCommandType::kStop:
      stopMotor();
      deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kStopOk);
      return;
    case triwhirl::runtime::RuntimeCommandType::kSwingAbort:
      if (!swing_id_runner.active()) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kSwingAlreadyInactive);
        return;
      }
      {
        triwhirl::runtime::RuntimeReply reply{};
        reply.code = triwhirl::runtime::RuntimeReplyCode::kSwingAbortOk;
        publishReplyRecord(reply, !telemetry_enabled && !binary_dump_active);
      }
      finishSwingRun(swing_id_runner.abort(SwingIdStopReason::kExternalAbort));
      return;
    case triwhirl::runtime::RuntimeCommandType::kSwingStart: {
      if (swing_id_runner.active()) {
        deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kSwingAlreadyActive);
        return;
      }
      if (motorActive()) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kSwingStartRequiresMotorStopped);
        return;
      }
      if (!motor_config_valid || !encoder_sample_valid ||
          !wheel_state.velocity_valid || !imu_sample_valid || !gyro_bias_valid ||
          !attitude_state.valid || safety_latch.faulted()) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kSwingStartRequiresReadyState);
        return;
      }
      const LoggerStatus log_status = runtime_logger.status();
      const std::uint32_t needed_records =
          (swing_id_runner.config().max_duration_us +
           triwhirl::log::kTwLogSamplePeriodUs - 1U) /
          triwhirl::log::kTwLogSamplePeriodUs;
      if (log_status.state != triwhirl::log::LoggerState::kRecording ||
          log_status.max_records < needed_records) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kSwingStartRequiresLogCapacity);
        return;
      }
      const std::uint32_t now_us =
          static_cast<std::uint32_t>(esp_timer_get_time());
      if (!swing_id_runner.start(currentSwingInput(now_us))) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kSwingStartRejected);
        return;
      }
      setSwingCriticalWindow(false);
      vq_command_v = clampFinite(swing_id_runner.output().desired_vq_v,
                                 -kMotorVectorLimitV, kMotorVectorLimitV);
      motor_mode = MotorMode::kFoc;
      deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kSwingStartOk);
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
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kSwingConfigUsageError);
        return;
      }
      deferRuntimeReply(makeSwingStatusReply(
          triwhirl::runtime::RuntimeReplyCode::kSwingConfigOk));
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kTimingReset:
      resetTimingStats();
      deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kTimingResetOk);
      return;
    case triwhirl::runtime::RuntimeCommandType::kTimingProfileOn:
      runtime_timing_profile.enabled = false;
      encoder.setTimingProfileEnabled(false);
      imu.setTimingProfileEnabled(false);
      resetRuntimeTimingProfile();
      encoder.setTimingProfileEnabled(true);
      imu.setTimingProfileEnabled(true);
      runtime_timing_profile.enabled = true;
      deferRuntimeReply(
          triwhirl::runtime::RuntimeReplyCode::kTimingProfileOnOk);
      return;
    case triwhirl::runtime::RuntimeCommandType::kTimingProfileOff: {
      runtime_timing_profile.enabled = false;
      encoder.setTimingProfileEnabled(false);
      imu.setTimingProfileEnabled(false);
      triwhirl::runtime::RuntimeReply reply{};
      reply.code = triwhirl::runtime::RuntimeReplyCode::kTimingProfileOffOk;
      publishReplyRecord(reply, false);
      publishRuntimeTimingProfile(promptAllowedNow());
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kTimingProfileReset:
      resetRuntimeTimingProfile();
      deferRuntimeReply(
          triwhirl::runtime::RuntimeReplyCode::kTimingProfileResetOk);
      return;
    case triwhirl::runtime::RuntimeCommandType::kFaultClear:
      stopMotor();
      if (!safety_latch.faulted()) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kFaultAlreadyClear);
        return;
      }
      if (!faultClearReady()) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kFaultClearRejected);
        return;
      }
      safety_latch.clear();
      deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kFaultClearOk);
      return;
    case triwhirl::runtime::RuntimeCommandType::kTelemetryOn:
      telemetry_enabled = true;
      last_telemetry_us = static_cast<std::uint32_t>(esp_timer_get_time());
      deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kTelemetryOnOk);
      return;
    case triwhirl::runtime::RuntimeCommandType::kTelemetryOff:
      telemetry_enabled = false;
      deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kTelemetryOffOk);
      return;
    case triwhirl::runtime::RuntimeCommandType::kMotorVq: {
      const float requested_vq = clampFinite(
          command.payload.motor_vq.volts, -kMotorVectorLimitV,
          kMotorVectorLimitV);
      if (std::fabs(requested_vq) < 1.0e-4F) {
        stopMotor();
        deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kMotorStopOk);
        return;
      }
      if (deferMotorStartFailure()) return;
      if (!motor_config_valid) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kMotorNotConfigured);
        return;
      }
      vq_command_v = requested_vq;
      motor_mode = MotorMode::kFoc;
      triwhirl::runtime::RuntimeReply reply{};
      reply.code = triwhirl::runtime::RuntimeReplyCode::kMotorFocOk;
      reply.float0 = vq_command_v;
      deferRuntimeReply(reply);
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kMotorConfig: {
      MotorElectricalConfig config{};
      config.pole_pairs = command.payload.motor_config.pole_pairs;
      config.sensor_direction = command.payload.motor_config.sensor_direction;
      config.electrical_offset_rad = triwhirl::wrapElectricalAngle(
          command.payload.motor_config.electrical_offset_rad);
      if (!triwhirl::validMotorElectricalConfig(config)) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kInvalidMotorConfig);
        return;
      }
      stopMotor();
      motor_config = config;
      motor_config_valid = true;
      triwhirl::runtime::RuntimeReply reply{};
      reply.code = triwhirl::runtime::RuntimeReplyCode::kMotorConfigOk;
      reply.value0 = motor_config.pole_pairs;
      reply.value1 = motor_config.sensor_direction;
      reply.float0 = motor_config.electrical_offset_rad;
      deferRuntimeReply(reply);
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kMotorCalibrate: {
      if (deferMotorStartFailure()) return;
      stopMotor();
      if (!encoder_sample_valid) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kMotorCalibrationEncoderUnavailable);
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
      triwhirl::runtime::RuntimeReply reply{};
      reply.code = triwhirl::runtime::RuntimeReplyCode::kMotorCalibrationStarted;
      reply.float0 = calibration.amplitude_v;
      reply.float1 = calibration.electrical_hz;
      reply.float2 = calibration.electrical_turns;
      deferRuntimeReply(reply);
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
        deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kFieldStopped);
        return;
      }
      if (deferMotorStartFailure()) return;
      stopMotor();
      open_loop_hz = requested_hz;
      open_loop_amplitude_v = requested_amplitude;
      open_loop_angle_rad = 0.0F;
      motor_mode = MotorMode::kOpenLoop;
      triwhirl::runtime::RuntimeReply reply{};
      reply.code = triwhirl::runtime::RuntimeReplyCode::kFieldOk;
      reply.float0 = open_loop_hz;
      reply.float1 = open_loop_amplitude_v;
      deferRuntimeReply(reply);
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kAttitudeReset:
      if (command.payload.attitude_reset.use_accelerometer) {
        resetAttitudeFromAccel();
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kAttitudeResetFromAccelOk);
        return;
      }
      if (!std::isfinite(command.payload.attitude_reset.angle_rad)) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kInvalidAttitudeAngle);
        return;
      }
      attitude_estimator.reset(command.payload.attitude_reset.angle_rad, 0.0F);
      attitude_state = attitude_estimator.state();
      attitude_initialized = true;
      last_attitude_update_us = static_cast<std::uint32_t>(esp_timer_get_time());
      {
        triwhirl::runtime::RuntimeReply reply{};
        reply.code = triwhirl::runtime::RuntimeReplyCode::kAttitudeResetAngleOk;
        reply.float0 = command.payload.attitude_reset.angle_rad;
        deferRuntimeReply(reply);
      }
      return;
    case triwhirl::runtime::RuntimeCommandType::kImuCalibrate: {
      const std::uint32_t samples =
          startGyroCalibration(command.payload.imu_calibrate.samples);
      if (samples == 0U) {
        deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kImuUnavailable);
        return;
      }
      triwhirl::runtime::RuntimeReply reply{};
      reply.code = triwhirl::runtime::RuntimeReplyCode::kImuCalibrationStarted;
      reply.u32_0 = samples;
      deferRuntimeReply(reply);
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kImuMap: {
      ImuPlanarMap map{};
      map.accel_sin_axis = command.payload.imu_map.accel_sin_axis;
      map.accel_cos_axis = command.payload.imu_map.accel_cos_axis;
      map.gyro_axis = command.payload.imu_map.gyro_axis;
      map.accel_sin_sign = command.payload.imu_map.accel_sin_sign;
      map.accel_cos_sign = command.payload.imu_map.accel_cos_sign;
      map.gyro_sign = command.payload.imu_map.gyro_sign;
      if (!validImuMap(map)) {
        deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kInvalidImuMap);
        return;
      }
      imu_map = map;
      resetAttitudeFromAccel();
      triwhirl::runtime::RuntimeReply reply{};
      reply.code = triwhirl::runtime::RuntimeReplyCode::kImuMapOk;
      reply.value0 = imu_map.accel_sin_axis;
      reply.value1 = imu_map.accel_cos_axis;
      reply.value2 = imu_map.gyro_axis;
      reply.value3 = imu_map.accel_sin_sign;
      reply.value4 = imu_map.accel_cos_sign;
      reply.value5 = imu_map.gyro_sign;
      deferRuntimeReply(reply);
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kLogPrepare: {
      const float seconds = command.payload.log_prepare.seconds;
      if (motorActive()) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kLogPrepareRequiresMotorStopped);
        return;
      }
      if (!std::isfinite(seconds) || seconds <= 0.0F) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kLogPrepareSecondsInvalid);
        return;
      }
      const double records_d = std::ceil(
          static_cast<double>(seconds) * 1000000.0 /
          static_cast<double>(triwhirl::log::kTwLogSamplePeriodUs));
      if (records_d < 1.0 ||
          records_d > static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kLogPrepareDurationOutOfRange);
        return;
      }
      const std::uint32_t records = static_cast<std::uint32_t>(records_d);
      if (!runtime_logger.prepare(records)) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kLogPrepareRejected);
        return;
      }
      log_critical_window = false;
      triwhirl::runtime::RuntimeReply reply{};
      reply.code = triwhirl::runtime::RuntimeReplyCode::kLogPrepareOk;
      reply.u32_0 = records;
      reply.float0 = seconds;
      deferRuntimeReply(reply);
      return;
    }
    case triwhirl::runtime::RuntimeCommandType::kLogStart:
      if (!runtime_logger.start()) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kLogStartRequiresReady);
        return;
      }
      log_critical_window = false;
      deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kLogStartOk);
      return;
    case triwhirl::runtime::RuntimeCommandType::kLogCriticalOn:
      log_critical_window = true;
      runtime_logger.setFlashWritesAllowed(false);
      deferRuntimeReply(
          triwhirl::runtime::RuntimeReplyCode::kLogCriticalOnOk);
      return;
    case triwhirl::runtime::RuntimeCommandType::kLogCriticalOff:
      log_critical_window = false;
      runtime_logger.setFlashWritesAllowed(true);
      deferRuntimeReply(
          triwhirl::runtime::RuntimeReplyCode::kLogCriticalOffOk);
      return;
    case triwhirl::runtime::RuntimeCommandType::kLogStop:
      log_critical_window = false;
      runtime_logger.setFlashWritesAllowed(true);
      if (!runtime_logger.stop()) {
        deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kLogStopRejected);
        return;
      }
      deferRuntimeReply(triwhirl::runtime::RuntimeReplyCode::kLogStopOk);
      return;
    case triwhirl::runtime::RuntimeCommandType::kLogDump:
      if (binary_dump_active || log_dump_task != nullptr) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kLogDumpAlreadyActive);
        return;
      }
      if (!runtime_logger.complete()) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kLogDumpRequiresComplete);
        return;
      }
      if (motorActive()) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kLogDumpRequiresMotorStopped);
        return;
      }
      if (!triwhirl::ble::connected() || !triwhirl::ble::subscribed()) {
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kLogDumpRequiresBleSubscription);
        return;
      }
      telemetry_enabled = false;
      binary_dump_active = true;
      command_reply_deferred = true;
      if (xTaskCreatePinnedToCore(logDumpTask, "triwhirl_log_dump", 4096, nullptr,
                                  1, &log_dump_task, 0) != pdPASS) {
        binary_dump_active = false;
        log_dump_task = nullptr;
        deferRuntimeReply(
            triwhirl::runtime::RuntimeReplyCode::kLogDumpTaskFailed);
      }
      return;
    case triwhirl::runtime::RuntimeCommandType::kNone:
      return;
  }
}

void processOneSupervisorInput() {
  triwhirl::runtime::SupervisorInputEvent event{};
  if (!triwhirl::runtime::tryReceiveSupervisorInput(&event)) return;
  command_reply_deferred = false;
  executeRuntimeCommand(event.runtime_command);
  if (!command_reply_deferred && !swing_id_runner.active()) printPrompt();
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
    if (profile) recordControlPeriod(start_us);

    std::uint32_t encoder_sequence = 0U;
    const bool encoder_dispatched =
        triwhirl::runtime::dispatchEncoderAcquisition(&encoder_sequence);

    const std::int64_t imu_begin_us = esp_timer_get_time();
    const bool imu_sampled = sampleImu();
    const std::int64_t attitude_begin_us = esp_timer_get_time();
    if (imu_sampled) updateAttitude(loop_us);
    const std::int64_t imu_end_us = esp_timer_get_time();
    if (profile) {
      recordRuntimeTimingStage(RuntimeTimingStage::kImuAttitude, imu_begin_us,
                               imu_end_us);
      if (imu_sampled) {
        recordLocalTiming(&attitude_math_timing, attitude_begin_us, imu_end_us);
      }
    }

    const std::int64_t encoder_join_begin_us = imu_end_us;
    if (encoder_dispatched) collectAndCommitEncoder(encoder_sequence, loop_us);
    else noteEncoderMiss();
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
      vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
  }
}

}  // namespace

void triwhirl::runtime::realtimeControlTask(void* opaque) {
  realtimeControlTaskImpl(opaque);
}

extern "C" void app_main(void) {
  if (!initConsole()) {
    safety_latch.trip(SafetyFault::kStartup);
    return;
  }

  console_tx_stream = xStreamBufferCreate(kConsoleTxBufferBytes, 1U);
  if (console_tx_stream == nullptr) {
    safety_latch.trip(SafetyFault::kStartup);
    return;
  }
  if (xTaskCreatePinnedToCore(consoleTxTask, "triwhirl_uart_tx", 4096, nullptr,
                              2, nullptr, 0) != pdPASS) {
    safety_latch.trip(SafetyFault::kStartup);
    return;
  }

  if (!triwhirl::ble::init()) {
    consoleWrite("WARN BLE init failed; Web Bluetooth unavailable\r\n");
  }
  if (!runtime_logger.init()) {
    consoleWrite("WARN TWLG init failed; binary runtime logging unavailable\r\n");
  }

  i2c_master_bus_handle_t encoder_bus = nullptr;
  if (!initRuntimeEncoderBus(&encoder_bus) ||
      !encoder.init(encoder_bus, triwhirl::board::kAs5600I2cAddress)) {
    safety_latch.trip(SafetyFault::kStartup);
    consoleWrite("FATAL fault=startup AS5600 I2C init failed\r\n");
    return;
  }

  i2c_master_bus_handle_t imu_bus = nullptr;
  imu_ready = initRuntimeImuBus(&imu_bus) &&
              imu.init(imu_bus, triwhirl::board::kMpu6050I2cAddress);
  if (!imu_ready) {
    consoleWrite("WARN MPU6050 init failed; IMU functions unavailable\r\n");
  }

  if (!bridge.init(triwhirl::board::kMotorIn1Gpio,
                   triwhirl::board::kMotorIn2Gpio,
                   triwhirl::board::kMotorIn3Gpio, kPwmFrequencyHz,
                   triwhirl::board::kMotorBusNominalV)) {
    safety_latch.trip(SafetyFault::kActuator);
    consoleWrite("FATAL fault=actuator MCPWM bridge init failed\r\n");
    return;
  }

  bridge.stopZeroVector();
  const std::uint32_t now_us = static_cast<std::uint32_t>(esp_timer_get_time());
  sampleEncoder(now_us);
  refreshEncoderHealth();
  if (imu_ready) {
    sampleImu();
    const std::uint32_t calibration_samples =
        startGyroCalibration(kDefaultGyroCalibrationSamples);
    if (calibration_samples > 0U) {
      consolePrintf("OK imu gyro calibration started samples=%lu\r\n",
                    static_cast<unsigned long>(calibration_samples));
    }
  }
  last_motor_update_us = now_us;
  last_telemetry_us = now_us;

  consoleWrite("TriWhirl deterministic motor + IMU + attitude runtime ready\r\n");
  consoleWrite("ESP32 owns swing-identification realtime decisions; host/BLE is supervisory only\r\n");
  consoleWrite("TWLG is the authoritative 1 kHz identification time base; BLE events are observability only\r\n");
  consoleWrite("telemetry is off by default; use 'telemetry on' only for diagnostic streaming\r\n");
  printStatus();
  printLogStatus();
  printSwingStatus();
  printHelp();
  printSwingHelp();
  printPrompt();

  if (xTaskCreatePinnedToCore(triwhirl::runtime::realtimeControlTask,
                              "triwhirl_control", 8192, nullptr,
                              configMAX_PRIORITIES - 2, nullptr, 1) != pdPASS) {
    safety_latch.trip(SafetyFault::kStartup);
    stopMotor();
    consoleWrite("FATAL fault=startup control task creation failed\r\n");
    return;
  }

  vTaskDelete(nullptr);
}
