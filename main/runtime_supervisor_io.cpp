#include "runtime_supervisor_io.hpp"

#include <cstdio>
#include <cstring>
#include <type_traits>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "runtime_balance.hpp"
#include "runtime_command_parser.hpp"
#include "runtime_diagnostics.hpp"
#include "runtime_imu_acquisition.hpp"
#include "runtime_snapshot.hpp"
#include "triwhirl/ble_transport.hpp"
#include "triwhirl/runtime_logger.hpp"
#include "triwhirl/safety.hpp"
#include "triwhirl/swing_id.hpp"

namespace triwhirl::runtime {
namespace {

constexpr std::uint32_t kSupervisorPollPeriodMs = 5U;
constexpr UBaseType_t kSupervisorQueueDepth = 4U;
// One FIFO preserves Core-1 publication order across synchronous replies,
// asynchronous state events, telemetry, and profile summaries. Timing-profile
// status is the largest normal burst, so keep explicit headroom above it.
constexpr UBaseType_t kRuntimeEgressQueueDepth = 32U;

enum class RuntimeEgressType : std::uint8_t {
  kReply = 0,
  kStateEvent,
  kTelemetry,
  kProfileReport,
};

union RuntimeEgressPayload {
  RuntimeReply reply;
  RuntimeStateEvent state_event;
  RuntimeTelemetryFrame telemetry;
  RuntimeProfileReport profile_report;

  constexpr RuntimeEgressPayload() : reply{} {}
};

struct RuntimeEgressRecord {
  RuntimeEgressType type = RuntimeEgressType::kReply;
  RuntimeEgressPayload payload{};
};

static_assert(std::is_trivially_copyable_v<RuntimeEgressRecord>,
              "RuntimeEgressRecord must remain trivially copyable");
static_assert(sizeof(RuntimeEgressRecord) <= 160U,
              "RuntimeEgressRecord grew beyond the bounded queue budget");

struct CommandInputState {
  char line[kSupervisorCommandBytes]{};
  std::size_t length = 0U;
};

QueueHandle_t input_queue = nullptr;
QueueHandle_t egress_queue = nullptr;
TaskHandle_t supervisor_task = nullptr;
SupervisorWriteFn write_fn = nullptr;
void* write_context = nullptr;
CommandInputState uart_development_input{};
CommandInputState ble_gatt_input{};
std::uint32_t egress_dropped = 0U;

void writeBytes(const char* data, const std::size_t length) {
  if (write_fn != nullptr && data != nullptr && length > 0U) {
    write_fn(write_context, data, length);
  }
}

void writeText(const char* text) {
  if (text != nullptr) {
    writeBytes(text, std::strlen(text));
  }
}

void writeFormatted(const char* const buffer, const int length,
                    const std::size_t capacity) {
  if (buffer == nullptr || length <= 0 || capacity == 0U) {
    return;
  }
  const std::size_t count = static_cast<std::size_t>(length) < capacity
                                ? static_cast<std::size_t>(length)
                                : capacity - 1U;
  writeBytes(buffer, count);
}

void writePromptFromSnapshot() {
  RuntimeSnapshot snapshot{};
  if (readLatestRuntimeSnapshot(&snapshot) && !snapshot.telemetry_enabled &&
      !snapshot.log_dump_active && !snapshot.swing_active) {
    writeText("> ");
  }
}

bool publishCommand(const SupervisorInputEvent& event) {
  if (input_queue == nullptr) {
    return false;
  }
  if (xQueueSend(input_queue, &event, 0) != pdTRUE) {
    writeText("ERR command mailbox full\r\n");
    return false;
  }
  return true;
}

bool publishEgress(const RuntimeEgressRecord& record) {
  if (egress_queue == nullptr || xQueueSend(egress_queue, &record, 0) != pdTRUE) {
    ++egress_dropped;
    return false;
  }
  return true;
}

void writeRuntimeSnapshotUnavailable() {
  writeText("ERR runtime snapshot unavailable\r\n");
}

const char* motorModeName(const std::uint8_t mode) {
  switch (mode) {
    case 0U: return "stopped";
    case 1U: return "open_loop";
    case 2U: return "foc";
    case 3U: return "calibrating";
    default: return "unknown";
  }
}

const char* timingProfileStageName(const RuntimeTimingProfileStageId stage) {
  switch (stage) {
    case RuntimeTimingProfileStageId::kEncoder: return "encoder";
    case RuntimeTimingProfileStageId::kImuAttitude: return "imu_attitude";
    case RuntimeTimingProfileStageId::kSafetySwing: return "safety_swing";
    case RuntimeTimingProfileStageId::kMotor: return "motor";
    case RuntimeTimingProfileStageId::kLog: return "log";
    case RuntimeTimingProfileStageId::kConsole: return "console";
    case RuntimeTimingProfileStageId::kTelemetry: return "telemetry";
    case RuntimeTimingProfileStageId::kLoop: return "loop";
    case RuntimeTimingProfileStageId::kEncoderI2cRaw: return "encoder_i2c_raw";
    case RuntimeTimingProfileStageId::kEncoderI2cStatus: return "encoder_i2c_status";
    case RuntimeTimingProfileStageId::kMpuI2c: return "mpu_i2c";
    case RuntimeTimingProfileStageId::kMpuDecode: return "mpu_decode";
  }
  return "unknown";
}

void formatSwingStatusPayload(const RuntimeReply& reply) {
  char buffer[768];
  const auto state = static_cast<triwhirl::SwingIdState>(reply.value0);
  const auto reason = static_cast<triwhirl::SwingIdStopReason>(reply.value1);
  const auto vertex = static_cast<triwhirl::SwingIdVertex>(reply.value5);
  const int flags = reply.value6;
  const int length = std::snprintf(
      buffer, sizeof(buffer),
      "swing,state=%s,reason=%s,captures=%lu,target=%lu,half_cycle=%lu,vertex=%s,error_deg=%.3f,vq_v=%.3f,pump=%d,probe=%d,critical=%d,event_drops=%lu,pump_low=%.3f,pump_high=%.3f,capture_deg=%.3f,exit_deg=%.3f,rearm_deg=%.3f,probe_ms=%.3f,rate_switch=%.6f,polarity=%d,vertex_a_deg=%.3f,max_s=%.3f\r\n",
      triwhirl::swingIdStateName(state),
      triwhirl::swingIdStopReasonName(reason),
      static_cast<unsigned long>(reply.value2),
      static_cast<unsigned long>(reply.value3),
      static_cast<unsigned long>(reply.value4),
      triwhirl::swingIdVertexName(vertex), reply.float0, reply.float1,
      (flags & 0x01) != 0 ? 1 : 0, (flags & 0x02) != 0 ? 1 : 0,
      (flags & 0x04) != 0 ? 1 : 0,
      static_cast<unsigned long>(reply.value7), reply.float2, reply.float3,
      reply.float4, reply.float5, reply.float6,
      static_cast<float>(reply.u32_0) * 1.0e-3F, reply.float7, reply.value8,
      reply.float8, static_cast<float>(reply.u32_1) * 1.0e-6F);
  writeFormatted(buffer, length, sizeof(buffer));
}

void formatBalanceStatusPayload(const RuntimeReply& reply) {
  char buffer[512];
  const int length = std::snprintf(
      buffer, sizeof(buffer),
      "balance,configured=%d,active=%d,k_theta=%.9g,k_rate=%.9g,k_wheel=%.9g,theta_ref_deg=%.3f,capture_deg=%.3f,fall_deg=%.3f,vq_limit_v=%.3f,wheel_limit_rad_s=%.3f,error_deg=%.3f\r\n",
      reply.value0, reply.value1, reply.float0, reply.float1, reply.float2,
      reply.float3, reply.float4, reply.float5, reply.float6, reply.float7,
      reply.float8);
  writeFormatted(buffer, length, sizeof(buffer));
}

void formatRuntimeReply(const RuntimeReply& reply) {
  char buffer[320];
  int length = 0;
  switch (reply.code) {
    case RuntimeReplyCode::kNone:
      break;
    case RuntimeReplyCode::kSwingOwnsRealtime:
      writeText("ERR swing experiment owns realtime actuation; use 'swing abort' first\r\n");
      break;
    case RuntimeReplyCode::kMotorStopOk:
      writeText("OK motor stop\r\n");
      break;
    case RuntimeReplyCode::kStopOk:
      writeText("OK stop\r\n");
      break;
    case RuntimeReplyCode::kSwingAlreadyInactive:
      writeText("OK swing already inactive\r\n");
      break;
    case RuntimeReplyCode::kSwingAbortOk:
      writeText("OK swing abort\r\n");
      break;
    case RuntimeReplyCode::kSwingAlreadyActive:
      writeText("ERR swing already active\r\n");
      break;
    case RuntimeReplyCode::kSwingStartRequiresMotorStopped:
      writeText("ERR swing start requires motor stopped\r\n");
      break;
    case RuntimeReplyCode::kSwingStartRequiresReadyState:
      writeText("ERR swing start requires motor config, encoder/wheel, calibrated IMU, valid attitude, and clear safety\r\n");
      break;
    case RuntimeReplyCode::kSwingStartRequiresLogCapacity:
      writeText("ERR swing start requires active TWLG recording with capacity for max duration\r\n");
      break;
    case RuntimeReplyCode::kSwingStartRejected:
      writeText("ERR swing start rejected\r\n");
      break;
    case RuntimeReplyCode::kSwingStartOk:
      writeText("OK swing start\r\n");
      break;
    case RuntimeReplyCode::kSwingConfigUsageError:
      writeText("ERR usage: swing config <captures> <pump_low_v> <pump_high_v> <capture_deg> <exit_deg> <rearm_deg> <probe_ms> <rate_switch_rad_s> <polarity> <vertex_a_deg> <max_s>\r\n");
      break;
    case RuntimeReplyCode::kSwingConfigOk:
      writeText("OK swing config\r\n");
      formatSwingStatusPayload(reply);
      break;
    case RuntimeReplyCode::kSwingStatus:
      formatSwingStatusPayload(reply);
      break;
    case RuntimeReplyCode::kSwingTransitionEvent: {
      const auto state = static_cast<triwhirl::SwingIdState>(reply.value0);
      const auto vertex = static_cast<triwhirl::SwingIdVertex>(reply.value4);
      const auto reason = static_cast<triwhirl::SwingIdStopReason>(reply.value5);
      length = std::snprintf(
          buffer, sizeof(buffer),
          "event,swing_id,state=%s,captures=%lu,target=%lu,half_cycle=%lu,vertex=%s,error_deg=%.3f,vq_v=%.3f,reason=%s,fault_mask=0x%08lx\r\n",
          triwhirl::swingIdStateName(state),
          static_cast<unsigned long>(reply.value1),
          static_cast<unsigned long>(reply.value2),
          static_cast<unsigned long>(reply.value3),
          triwhirl::swingIdVertexName(vertex), reply.float0, reply.float1,
          triwhirl::swingIdStopReasonName(reason),
          static_cast<unsigned long>(reply.u32_0));
      writeFormatted(buffer, length, sizeof(buffer));
      break;
    }
    case RuntimeReplyCode::kBalanceConfigRejected:
      writeText("ERR balance config rejected; motor must be stopped and parameters must be finite/in range\r\n");
      break;
    case RuntimeReplyCode::kBalanceConfigOk:
      writeText("OK balance config\r\n");
      formatBalanceStatusPayload(reply);
      break;
    case RuntimeReplyCode::kBalanceStartRejected:
      length = std::snprintf(
          buffer, sizeof(buffer), "ERR balance start rejected reason=%s\r\n",
          balanceStartFailureName(
              static_cast<BalanceStartFailure>(reply.value0)));
      writeFormatted(buffer, length, sizeof(buffer));
      break;
    case RuntimeReplyCode::kBalanceStartOk:
      length = std::snprintf(buffer, sizeof(buffer),
                             "OK balance start vq_v=%.6f\r\n", reply.float0);
      writeFormatted(buffer, length, sizeof(buffer));
      break;
    case RuntimeReplyCode::kBalanceStopOk:
      writeText("OK balance stop\r\n");
      break;
    case RuntimeReplyCode::kBalanceStatus:
      formatBalanceStatusPayload(reply);
      break;
    case RuntimeReplyCode::kTimingResetOk:
      writeText("OK timing reset\r\n");
      break;
    case RuntimeReplyCode::kTimingProfileOnOk:
      writeText("OK timing profile on\r\n");
      break;
    case RuntimeReplyCode::kTimingProfileOffOk:
      writeText("OK timing profile off\r\n");
      break;
    case RuntimeReplyCode::kTimingProfileResetOk:
      writeText("OK timing profile reset\r\n");
      break;
    case RuntimeReplyCode::kTimingProfileHeader:
      length = std::snprintf(
          buffer, sizeof(buffer),
          "timing_profile,enabled=%d,samples=%llu,clock=esp_timer_us\r\n",
          reply.value0, static_cast<unsigned long long>(reply.wide0));
      writeFormatted(buffer, length, sizeof(buffer));
      break;
    case RuntimeReplyCode::kTimingProfileStage: {
      const double mean_us = reply.wide0 > 0U
                                 ? static_cast<double>(reply.wide1) /
                                       static_cast<double>(reply.wide0)
                                 : 0.0;
      const auto stage = static_cast<RuntimeTimingProfileStageId>(reply.value0);
      length = std::snprintf(
          buffer, sizeof(buffer),
          "timing_profile_stage,name=%s,count=%llu,mean_us=%.3f,min_us=%lu,max_us=%lu\r\n",
          timingProfileStageName(stage),
          static_cast<unsigned long long>(reply.wide0), mean_us,
          static_cast<unsigned long>(reply.wide0 > 0U ? reply.u32_0 : 0U),
          static_cast<unsigned long>(reply.wide0 > 0U ? reply.u32_1 : 0U));
      writeFormatted(buffer, length, sizeof(buffer));
      break;
    }
    case RuntimeReplyCode::kTimingProfileEnd:
      writeText("timing_profile_end\r\n");
      break;
    case RuntimeReplyCode::kFaultAlreadyClear:
      writeText("OK fault already clear\r\n");
      break;
    case RuntimeReplyCode::kFaultClearRejected:
      writeText("ERR fault clear rejected; fault cause is still present\r\n");
      break;
    case RuntimeReplyCode::kFaultClearOk:
      writeText("OK fault clear\r\n");
      break;
    case RuntimeReplyCode::kTelemetryOnOk:
      writeText("OK telemetry on\r\n");
      break;
    case RuntimeReplyCode::kTelemetryOffOk:
      writeText("OK telemetry off\r\n");
      break;
    case RuntimeReplyCode::kSafetyFaultLatched:
      length = std::snprintf(
          buffer, sizeof(buffer),
          "ERR safety fault latched first=%s mask=0x%08lx; use 'fault status'\r\n",
          triwhirl::safetyFaultName(static_cast<triwhirl::SafetyFault>(reply.value0)),
          static_cast<unsigned long>(reply.u32_0));
      writeFormatted(buffer, length, sizeof(buffer));
      break;
    case RuntimeReplyCode::kEncoderUnavailable:
      writeText("ERR encoder read unavailable\r\n");
      break;
    case RuntimeReplyCode::kInvalidRuntimeNumeric:
      writeText("ERR invalid runtime numeric state\r\n");
      break;
    case RuntimeReplyCode::kInvalidMotorConfig:
      writeText("ERR invalid motor config\r\n");
      break;
    case RuntimeReplyCode::kMotorConfigOk:
      length = std::snprintf(
          buffer, sizeof(buffer),
          "OK motor config pole_pairs=%d sensor_dir=%d offset_rad=%.6f\r\n",
          reply.value0, reply.value1, reply.float0);
      writeFormatted(buffer, length, sizeof(buffer));
      break;
    case RuntimeReplyCode::kMotorNotConfigured:
      writeText("ERR motor is not calibrated/configured\r\n");
      break;
    case RuntimeReplyCode::kMotorFocOk:
      length = std::snprintf(buffer, sizeof(buffer),
                             "OK motor FOC vq_v=%.6f\r\n", reply.float0);
      writeFormatted(buffer, length, sizeof(buffer));
      break;
    case RuntimeReplyCode::kMotorCalibrationEncoderUnavailable:
      writeText("ERR motor calibrate: encoder read unavailable\r\n");
      break;
    case RuntimeReplyCode::kMotorCalibrationStarted:
      length = std::snprintf(
          buffer, sizeof(buffer),
          "OK motor calibration started amp_v=%.3f e_hz=%.3f turns=%.3f\r\n",
          reply.float0, reply.float1, reply.float2);
      writeFormatted(buffer, length, sizeof(buffer));
      break;
    case RuntimeReplyCode::kFieldStopped:
      writeText("OK field stopped\r\n");
      break;
    case RuntimeReplyCode::kFieldOk:
      length = std::snprintf(buffer, sizeof(buffer),
                             "OK field e_hz=%.6f amp_v=%.6f\r\n",
                             reply.float0, reply.float1);
      writeFormatted(buffer, length, sizeof(buffer));
      break;
    case RuntimeReplyCode::kAttitudeResetFromAccelOk:
      writeText("OK attitude reset from accelerometer\r\n");
      break;
    case RuntimeReplyCode::kInvalidAttitudeAngle:
      writeText("ERR invalid attitude angle\r\n");
      break;
    case RuntimeReplyCode::kAttitudeResetAngleOk:
      length = std::snprintf(buffer, sizeof(buffer),
                             "OK attitude reset angle_rad=%.6f\r\n",
                             reply.float0);
      writeFormatted(buffer, length, sizeof(buffer));
      break;
    case RuntimeReplyCode::kImuUnavailable:
      writeText("ERR imu unavailable\r\n");
      break;
    case RuntimeReplyCode::kImuCalibrationStarted:
      length = std::snprintf(buffer, sizeof(buffer),
                             "OK imu gyro calibration started samples=%lu\r\n",
                             static_cast<unsigned long>(reply.u32_0));
      writeFormatted(buffer, length, sizeof(buffer));
      break;
    case RuntimeReplyCode::kInvalidImuMap:
      writeText("ERR invalid imu map\r\n");
      break;
    case RuntimeReplyCode::kImuMapOk:
      length = std::snprintf(buffer, sizeof(buffer),
                             "OK imu map %d %d %d %d %d %d\r\n",
                             reply.value0, reply.value1, reply.value2,
                             reply.value3, reply.value4, reply.value5);
      writeFormatted(buffer, length, sizeof(buffer));
      break;
    case RuntimeReplyCode::kLogPrepareRequiresMotorStopped:
      writeText("ERR log prepare requires motor stopped\r\n");
      break;
    case RuntimeReplyCode::kLogPrepareSecondsInvalid:
      writeText("ERR log prepare seconds must be > 0\r\n");
      break;
    case RuntimeReplyCode::kLogPrepareDurationOutOfRange:
      writeText("ERR log prepare duration out of range\r\n");
      break;
    case RuntimeReplyCode::kLogPrepareRejected:
      writeText("ERR log prepare rejected; check log status/capacity\r\n");
      break;
    case RuntimeReplyCode::kLogPrepareOk:
      length = std::snprintf(
          buffer, sizeof(buffer),
          "OK log prepare records=%lu seconds=%.3f; erase in background\r\n",
          static_cast<unsigned long>(reply.u32_0), reply.float0);
      writeFormatted(buffer, length, sizeof(buffer));
      break;
    case RuntimeReplyCode::kLogStartRequiresReady:
      writeText("ERR log start requires state=ready\r\n");
      break;
    case RuntimeReplyCode::kLogStartOk:
      writeText("OK log start sample_us=1000 record_bytes=32\r\n");
      break;
    case RuntimeReplyCode::kLogCriticalOnOk:
      writeText("OK log critical on; flash programming paused\r\n");
      break;
    case RuntimeReplyCode::kLogCriticalOffOk:
      writeText("OK log critical off; flash programming resumed\r\n");
      break;
    case RuntimeReplyCode::kLogStopRejected:
      writeText("ERR log stop rejected; check log status\r\n");
      break;
    case RuntimeReplyCode::kLogStopOk:
      writeText("OK log stopping; SRAM is draining and header will finalize\r\n");
      break;
    case RuntimeReplyCode::kLogDumpAlreadyActive:
      writeText("ERR log dump already active\r\n");
      break;
    case RuntimeReplyCode::kLogDumpRequiresComplete:
      writeText("ERR log dump requires state=complete\r\n");
      break;
    case RuntimeReplyCode::kLogDumpRequiresMotorStopped:
      writeText("ERR log dump requires motor stopped\r\n");
      break;
    case RuntimeReplyCode::kLogDumpRequiresBleSubscription:
      writeText("ERR log dump requires BLE notify subscription\r\n");
      break;
    case RuntimeReplyCode::kLogDumpTaskFailed:
      writeText("ERR log dump task creation failed\r\n");
      break;
  }

  if (reply.prompt_after) {
    writeText("> ");
  }
}

void formatRuntimeStateEvent(const RuntimeStateEvent& event) {
  char buffer[256];
  int length = 0;
  switch (event.type) {
    case RuntimeStateEventType::kNone:
      return;
    case RuntimeStateEventType::kFaultLatched:
      length = std::snprintf(
          buffer, sizeof(buffer), "FAULT,code=%s,mask=0x%08lx\r\n",
          triwhirl::safetyFaultName(static_cast<triwhirl::SafetyFault>(event.value0)),
          static_cast<unsigned long>(event.u32_0));
      break;
    case RuntimeStateEventType::kMotorCalibrationEncoderUnavailable:
      writeText("ERR motor calibration: encoder read unavailable\r\n");
      return;
    case RuntimeStateEventType::kMotorCalibrationNoMotion:
      writeText("ERR motor calibration: no usable mechanical motion\r\n");
      return;
    case RuntimeStateEventType::kMotorCalibrationPolePairInvalid:
      length = std::snprintf(
          buffer, sizeof(buffer),
          "ERR motor calibration: pole-pair estimate %.3f is invalid\r\n",
          event.float0);
      break;
    case RuntimeStateEventType::kMotorCalibrationConfigInvalid:
      writeText("ERR motor calibration: generated configuration is invalid\r\n");
      return;
    case RuntimeStateEventType::kMotorCalibrationComplete:
      length = std::snprintf(
          buffer, sizeof(buffer),
          "OK motor calibrated pole_pairs=%d sensor_dir=%d offset_rad=%.6f estimate=%.3f\r\n",
          event.value0, event.value1, event.float0, event.float1);
      break;
    case RuntimeStateEventType::kFocStoppedConfigUnavailable:
      writeText("ERR FOC stopped: motor configuration unavailable\r\n");
      return;
    case RuntimeStateEventType::kFocStoppedEncoderUnavailable:
      writeText("ERR FOC stopped: encoder unavailable\r\n");
      return;
    case RuntimeStateEventType::kImuCalibrationComplete:
      length = std::snprintf(
          buffer, sizeof(buffer),
          "OK imu gyro calibration bx=%.6f by=%.6f bz=%.6f rad_s\r\n",
          event.float0, event.float1, event.float2);
      break;
  }
  writeFormatted(buffer, length, sizeof(buffer));
}

void formatRuntimeTelemetry(const RuntimeTelemetryFrame& frame) {
  char buffer[768];
  const int length = std::snprintf(
      buffer, sizeof(buffer),
      "telemetry,%lu,%s,%.6f,%.6f,%.6f,%d,%d,%d,%u,%lld,%.6f,%.6f,%.6f,%.6f,%d,%lu,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%lu,%d,%.6f,%.6f,%.6f,%lu,%lu,%llu,%lu\r\n",
      static_cast<unsigned long>(frame.t_us), motorModeName(frame.motor_mode),
      frame.motor_vq_v, frame.motor_electrical_angle_rad,
      frame.motor_electrical_hz, frame.encoder_status_valid ? 1 : 0,
      frame.encoder_sample_valid ? 1 : 0,
      frame.encoder_magnet_detected ? 1 : 0,
      static_cast<unsigned>(frame.encoder_raw_count),
      static_cast<long long>(frame.encoder_unwrapped_count),
      frame.encoder_angle_rad, frame.encoder_unwrapped_rad,
      frame.encoder_velocity_rad_s, frame.encoder_instantaneous_velocity_rad_s,
      frame.encoder_velocity_valid ? 1 : 0,
      static_cast<unsigned long>(frame.encoder_read_errors),
      frame.imu_sample_valid ? 1 : 0, frame.imu_ax_mps2, frame.imu_ay_mps2,
      frame.imu_az_mps2, frame.imu_gx_rad_s, frame.imu_gy_rad_s,
      frame.imu_gz_rad_s, static_cast<unsigned long>(frame.imu_read_errors),
      frame.attitude_valid ? 1 : 0, frame.attitude_angle_rad,
      frame.attitude_rate_rad_s, frame.attitude_accel_weight,
      static_cast<unsigned long>(frame.timing_last_exec_us),
      static_cast<unsigned long>(frame.timing_max_exec_us),
      static_cast<unsigned long long>(frame.timing_overruns),
      static_cast<unsigned long>(frame.safety_fault_mask));
  writeFormatted(buffer, length, sizeof(buffer));
}

void formatRuntimeProfileReport(const RuntimeProfileReport& report) {
  char buffer[1024];
  const double attitude_mean_us =
      report.attitude_count > 0U
          ? static_cast<double>(report.attitude_total_us) /
                static_cast<double>(report.attitude_count)
          : 0.0;
  const int length = std::snprintf(
      buffer, sizeof(buffer),
      "parallel_profile,requests=%llu,completions=%llu,dispatch_failures=%llu,read_failures=%llu,stale_results=%llu,join_timeouts=%llu,max_consecutive_misses=%lu,attitude_count=%llu,attitude_mean_us=%.3f,attitude_min_us=%lu,attitude_max_us=%lu,period_lt900=%llu,period_900_949=%llu,period_950_999=%llu,period_1000_1049=%llu,period_1050_1099=%llu,period_1100_1249=%llu,period_1250_1499=%llu,period_ge1500=%llu\r\n",
      static_cast<unsigned long long>(report.requests),
      static_cast<unsigned long long>(report.completions),
      static_cast<unsigned long long>(report.dispatch_failures),
      static_cast<unsigned long long>(report.read_failures),
      static_cast<unsigned long long>(report.stale_results),
      static_cast<unsigned long long>(report.join_timeouts),
      static_cast<unsigned long>(report.max_consecutive_misses),
      static_cast<unsigned long long>(report.attitude_count), attitude_mean_us,
      static_cast<unsigned long>(report.attitude_min_us),
      static_cast<unsigned long>(report.attitude_max_us),
      static_cast<unsigned long long>(report.period_lt900),
      static_cast<unsigned long long>(report.period_900_949),
      static_cast<unsigned long long>(report.period_950_999),
      static_cast<unsigned long long>(report.period_1000_1049),
      static_cast<unsigned long long>(report.period_1050_1099),
      static_cast<unsigned long long>(report.period_1100_1249),
      static_cast<unsigned long long>(report.period_1250_1499),
      static_cast<unsigned long long>(report.period_ge1500));
  writeFormatted(buffer, length, sizeof(buffer));
}

void formatRuntimeEgress(const RuntimeEgressRecord& record) {
  switch (record.type) {
    case RuntimeEgressType::kReply:
      formatRuntimeReply(record.payload.reply);
      break;
    case RuntimeEgressType::kStateEvent:
      formatRuntimeStateEvent(record.payload.state_event);
      break;
    case RuntimeEgressType::kTelemetry:
      formatRuntimeTelemetry(record.payload.telemetry);
      break;
    case RuntimeEgressType::kProfileReport:
      formatRuntimeProfileReport(record.payload.profile_report);
      break;
  }
}

void drainRuntimeEgress() {
  if (egress_queue == nullptr) {
    return;
  }
  RuntimeEgressRecord record{};
  while (xQueueReceive(egress_queue, &record, 0) == pdTRUE) {
    formatRuntimeEgress(record);
  }
}

void writeHelp() {
  static constexpr char kHelp[] =
      "commands:\r\n"
      "  motor calibrate [amplitude_v] [electrical_hz] [turns]\r\n"
      "  motor config <pole_pairs> <sensor_dir> <offset_rad>\r\n"
      "  motor vq <volts>\r\n"
      "  motor status\r\n"
      "  motor stop\r\n"
      "  imu status\r\n"
      "  imu calibrate [samples]\r\n"
      "  imu map <sin_axis> <cos_axis> <gyro_axis> <sin_sign> <cos_sign> <gyro_sign>\r\n"
      "  attitude status\r\n"
      "  attitude reset [angle_rad]\r\n"
      "  timing status\r\n"
      "  timing reset\r\n"
      "  fault status\r\n"
      "  fault clear\r\n"
      "  ble status\r\n"
      "  log status\r\n"
      "  log prepare [seconds]\r\n"
      "  log start\r\n"
      "  log critical <on|off>\r\n"
      "  log stop\r\n"
      "  log dump\r\n"
      "  field <electrical_hz> <amplitude_v>\r\n"
      "  stop\r\n"
      "  status\r\n"
      "  telemetry [on|off]\r\n"
      "  help\r\n"
      "  swing status\r\n"
      "  swing config <captures> <pump_low_v> <pump_high_v> <capture_deg> <exit_deg> <rearm_deg> <probe_ms> <rate_switch_rad_s> <polarity> <vertex_a_deg> <max_s>\r\n"
      "  swing start\r\n"
      "  swing abort\r\n"
      "  balance status\r\n"
      "  balance config <k_theta> <k_rate> <k_wheel> <theta_ref_deg> <capture_deg> <fall_deg> <vq_limit_v> <wheel_limit_rad_s>\r\n"
      "  balance start\r\n"
      "  balance stop\r\n"
      "  timing profile <status|on|off|reset>\r\n";
  writeText(kHelp);
}

bool handleSupervisorReadOnlyCommand(const char* const line) {
  if (line == nullptr) {
    return false;
  }

  if (std::strcmp(line, "help") == 0) {
    writeHelp();
    writePromptFromSnapshot();
    return true;
  }

  const bool ble_status = std::strcmp(line, "ble") == 0 ||
                          std::strcmp(line, "ble status") == 0;
  if (ble_status) {
    char buffer[160];
    const int length = std::snprintf(
        buffer, sizeof(buffer),
        "ble,connected=%d,subscribed=%d,rx_drop_bytes=%lu,tx_drop_bytes=%lu\r\n",
        triwhirl::ble::connected() ? 1 : 0,
        triwhirl::ble::subscribed() ? 1 : 0,
        static_cast<unsigned long>(triwhirl::ble::rxDroppedBytes()),
        static_cast<unsigned long>(triwhirl::ble::txDroppedBytes()));
    writeFormatted(buffer, length, sizeof(buffer));
    writePromptFromSnapshot();
    return true;
  }

  const bool aggregate_status = std::strcmp(line, "status") == 0 ||
                                std::strcmp(line, "motor status") == 0;
  const bool imu_status = std::strcmp(line, "imu") == 0 ||
                          std::strcmp(line, "imu status") == 0;
  const bool log_status = std::strcmp(line, "log") == 0 ||
                          std::strcmp(line, "log status") == 0;
  const bool attitude_status = std::strcmp(line, "attitude") == 0 ||
                               std::strcmp(line, "attitude status") == 0;
  const bool fault_status = std::strcmp(line, "fault") == 0 ||
                            std::strcmp(line, "fault status") == 0;
  const bool timing_status = std::strcmp(line, "timing") == 0 ||
                             std::strcmp(line, "timing status") == 0;
  const bool telemetry_status = std::strcmp(line, "telemetry") == 0;
  if (!aggregate_status && !imu_status && !log_status && !attitude_status &&
      !fault_status && !timing_status && !telemetry_status) {
    return false;
  }

  RuntimeSnapshot snapshot{};
  if (!readLatestRuntimeSnapshot(&snapshot)) {
    writeRuntimeSnapshotUnavailable();
    writePromptFromSnapshot();
    return true;
  }

  if (aggregate_status) {
    EncoderDiagnosticStatus encoder_health{};
    readEncoderDiagnosticStatus(&encoder_health);
    char buffer[1024];
    const auto first_fault =
        static_cast<triwhirl::SafetyFault>(snapshot.safety_first_fault);
    const int length = std::snprintf(
        buffer, sizeof(buffer),
        "status,mode=%s,telemetry=%d,vq_v=%.6f,e_hz=%.6f,amp_v=%.6f,config=%d,pole_pairs=%d,sensor_dir=%d,offset_rad=%.6f,e_angle_rad=%.6f,status_ok=%d,sample_ok=%d,mag=%d,ml=%d,mh=%d,raw=%u,unwrapped_count=%lld,angle_rad=%.6f,unwrapped_rad=%.6f,vel_rad_s=%.6f,vel_inst_rad_s=%.6f,vel_valid=%d,read_errors=%lu,imu_ok=%d,attitude_ok=%d,theta_rad=%.6f,theta_rate_rad_s=%.6f,ble_connected=%d,ble_subscribed=%d,fault_mask=0x%08lx,fault_first=%s\r\n",
        motorModeName(snapshot.motor_mode), snapshot.telemetry_enabled ? 1 : 0,
        snapshot.motor_vq_v, snapshot.motor_electrical_hz,
        snapshot.motor_amplitude_v, snapshot.motor_config_valid ? 1 : 0,
        snapshot.motor_pole_pairs, snapshot.motor_sensor_direction,
        snapshot.motor_offset_rad, snapshot.motor_electrical_angle_rad,
        encoder_health.status_ok ? 1 : 0,
        snapshot.encoder_sample_valid ? 1 : 0,
        encoder_health.magnet_detected ? 1 : 0,
        encoder_health.magnet_too_weak ? 1 : 0,
        encoder_health.magnet_too_strong ? 1 : 0,
        static_cast<unsigned>(snapshot.encoder_raw_count),
        static_cast<long long>(snapshot.encoder_unwrapped_count),
        snapshot.encoder_angle_rad, snapshot.encoder_unwrapped_rad,
        snapshot.encoder_velocity_rad_s,
        snapshot.encoder_instantaneous_velocity_rad_s,
        snapshot.encoder_velocity_valid ? 1 : 0,
        static_cast<unsigned long>(snapshot.encoder_read_errors),
        snapshot.imu_sample_valid ? 1 : 0,
        snapshot.attitude_valid ? 1 : 0, snapshot.attitude_angle_rad,
        snapshot.attitude_rate_rad_s, triwhirl::ble::connected() ? 1 : 0,
        triwhirl::ble::subscribed() ? 1 : 0,
        static_cast<unsigned long>(snapshot.safety_fault_mask),
        triwhirl::safetyFaultName(first_fault));
    writeFormatted(buffer, length, sizeof(buffer));
  } else if (imu_status) {
    const ImuAcquisitionStats imu_acq = imuAcquisitionStats();
    char buffer[896];
    const int length = std::snprintf(
        buffer, sizeof(buffer),
        "imu,ready=%d,sample_ok=%d,who_ok=%d,who=0x%02x,bias_valid=%d,calibrating=%d,ax=%.6f,ay=%.6f,az=%.6f,gx=%.6f,gy=%.6f,gz=%.6f,temp_c=%.3f,bx=%.6f,by=%.6f,bz=%.6f,map=%d:%d:%d:%d:%d:%d,read_errors=%lu,drdy_gpio=%d,drdy_probe_only=%d,drdy_edges=%llu,drdy_consumed=%llu,drdy_fallback_reads=%llu\r\n",
        snapshot.imu_ready ? 1 : 0, snapshot.imu_sample_valid ? 1 : 0,
        snapshot.imu_identity_valid ? 1 : 0,
        static_cast<unsigned>(snapshot.imu_who_am_i),
        snapshot.imu_bias_valid ? 1 : 0, snapshot.imu_calibrating ? 1 : 0,
        snapshot.imu_ax_mps2, snapshot.imu_ay_mps2, snapshot.imu_az_mps2,
        snapshot.imu_gx_rad_s, snapshot.imu_gy_rad_s, snapshot.imu_gz_rad_s,
        snapshot.imu_temperature_c, snapshot.imu_bias_x_rad_s,
        snapshot.imu_bias_y_rad_s, snapshot.imu_bias_z_rad_s,
        snapshot.imu_map_sin_axis, snapshot.imu_map_cos_axis,
        snapshot.imu_map_gyro_axis, snapshot.imu_map_sin_sign,
        snapshot.imu_map_cos_sign, snapshot.imu_map_gyro_sign,
        static_cast<unsigned long>(snapshot.imu_read_errors), imu_acq.drdy_gpio,
        imu_acq.drdy_probe_only ? 1 : 0,
        static_cast<unsigned long long>(imu_acq.drdy_edges),
        static_cast<unsigned long long>(imu_acq.drdy_consumed),
        static_cast<unsigned long long>(imu_acq.drdy_fallback_reads));
    writeFormatted(buffer, length, sizeof(buffer));
  } else if (log_status) {
    char buffer[512];
    const int length = std::snprintf(
        buffer, sizeof(buffer),
        "log,state=%s,partition_bytes=%lu,prepared_bytes=%lu,max_records=%lu,buffered_bytes=%lu,records_written=%lu,dropped_records=%lu,logical_bytes=%lu,flash_write=%d,critical=%d,dump_active=%d\r\n",
        triwhirl::log::loggerStateName(
            static_cast<triwhirl::log::LoggerState>(snapshot.log_state)),
        static_cast<unsigned long>(snapshot.log_partition_bytes),
        static_cast<unsigned long>(snapshot.log_prepared_bytes),
        static_cast<unsigned long>(snapshot.log_max_records),
        static_cast<unsigned long>(snapshot.log_buffered_bytes),
        static_cast<unsigned long>(snapshot.log_records_written),
        static_cast<unsigned long>(snapshot.log_dropped_records),
        static_cast<unsigned long>(snapshot.log_logical_bytes),
        snapshot.log_flash_writes_allowed ? 1 : 0,
        snapshot.log_critical_window ? 1 : 0,
        snapshot.log_dump_active ? 1 : 0);
    writeFormatted(buffer, length, sizeof(buffer));
  } else if (attitude_status) {
    char buffer[320];
    const int length = std::snprintf(
        buffer, sizeof(buffer),
        "attitude,initialized=%d,valid=%d,theta_rad=%.6f,rate_rad_s=%.6f,residual_bias_rad_s=%.6f,innovation=%.6f,accel_weight=%.6f,wheel_rate_rad_s=%.6f\r\n",
        snapshot.attitude_initialized ? 1 : 0,
        snapshot.attitude_valid ? 1 : 0,
        snapshot.attitude_angle_rad,
        snapshot.attitude_rate_rad_s,
        snapshot.attitude_residual_bias_rad_s,
        snapshot.attitude_innovation,
        snapshot.attitude_accel_weight,
        snapshot.wheel_rate_rad_s);
    writeFormatted(buffer, length, sizeof(buffer));
  } else if (fault_status) {
    char buffer[128];
    const auto first_fault =
        static_cast<triwhirl::SafetyFault>(snapshot.safety_first_fault);
    const int length = std::snprintf(
        buffer, sizeof(buffer),
        "fault,latched=%d,mask=0x%08lx,first=%s\r\n",
        snapshot.safety_faulted ? 1 : 0,
        static_cast<unsigned long>(snapshot.safety_fault_mask),
        triwhirl::safetyFaultName(first_fault));
    writeFormatted(buffer, length, sizeof(buffer));
  } else if (timing_status) {
    char buffer[384];
    const int length = std::snprintf(
        buffer, sizeof(buffer),
        "timing,target_us=%lu,hard_period_us=%lu,iterations=%llu,last_exec_us=%lu,max_exec_us=%lu,min_period_us=%lu,max_period_us=%lu,overruns=%llu,late_periods=%llu,uart_tx_drop_bytes=%lu,ble_rx_drop_bytes=%lu,ble_tx_drop_bytes=%lu\r\n",
        static_cast<unsigned long>(snapshot.timing_target_us),
        static_cast<unsigned long>(snapshot.timing_hard_period_us),
        static_cast<unsigned long long>(snapshot.timing_iterations),
        static_cast<unsigned long>(snapshot.timing_last_exec_us),
        static_cast<unsigned long>(snapshot.timing_max_exec_us),
        static_cast<unsigned long>(snapshot.timing_min_period_us),
        static_cast<unsigned long>(snapshot.timing_max_period_us),
        static_cast<unsigned long long>(snapshot.timing_overruns),
        static_cast<unsigned long long>(snapshot.timing_late_periods),
        static_cast<unsigned long>(snapshot.uart_tx_drop_bytes),
        static_cast<unsigned long>(triwhirl::ble::rxDroppedBytes()),
        static_cast<unsigned long>(triwhirl::ble::txDroppedBytes()));
    writeFormatted(buffer, length, sizeof(buffer));
  } else {
    writeText(snapshot.telemetry_enabled ? "telemetry=on\r\n"
                                         : "telemetry=off\r\n");
  }

  writePromptFromSnapshot();
  return true;
}

bool handleTypedRuntimeCommand(const char* const line) {
  const RuntimeCommandParseResult parsed = parseRuntimeCommand(line);
  switch (parsed.status) {
    case RuntimeCommandParseStatus::kNotMatched:
      return false;
    case RuntimeCommandParseStatus::kUsageError:
      writeText(parsed.error);
      writePromptFromSnapshot();
      return true;
    case RuntimeCommandParseStatus::kCommand: {
      SupervisorInputEvent event{};
      event.runtime_command = parsed.command;
      if (!publishCommand(event)) {
        writePromptFromSnapshot();
      }
      return true;
    }
  }
  return false;
}

void consumeBytes(const std::uint8_t* input, const std::size_t received,
                  CommandInputState& state) {
  if (input == nullptr) {
    return;
  }

  for (std::size_t index = 0U; index < received; ++index) {
    const char c = static_cast<char>(input[index]);

    if (c == '\r' || c == '\n') {
      if (state.length > 0U) {
        writeBytes("\r\n", 2U);
        state.line[state.length] = '\0';
        if (!handleSupervisorReadOnlyCommand(state.line) &&
            !handleTypedRuntimeCommand(state.line)) {
          writeText("ERR unknown command\r\n");
          writePromptFromSnapshot();
        }
        state.length = 0U;
      }
      continue;
    }

    if (c == '\b' || static_cast<unsigned char>(c) == 0x7FU) {
      if (state.length > 0U) {
        --state.length;
        writeBytes("\b \b", 3U);
      }
      continue;
    }

    if (c < 0x20 || static_cast<unsigned char>(c) > 0x7EU) {
      continue;
    }

    if (state.length + 1U < sizeof(state.line)) {
      state.line[state.length++] = c;
      writeBytes(&c, 1U);
      continue;
    }

    state.length = 0U;
    writeText("\r\nERR command too long\r\n");
    writePromptFromSnapshot();
  }
}

void supervisorIoTask(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  std::uint8_t input[64];

  while (true) {
    drainRuntimeEgress();

    const int uart_received = uart_read_bytes(UART_NUM_0, input, sizeof(input), 0);
    if (uart_received > 0) {
      consumeBytes(input, static_cast<std::size_t>(uart_received),
                   uart_development_input);
    }

    const std::size_t ble_received = triwhirl::ble::read(input, sizeof(input));
    if (ble_received > 0U) {
      consumeBytes(input, ble_received, ble_gatt_input);
    }

    drainRuntimeEgress();
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kSupervisorPollPeriodMs));
  }
}

}  // namespace

bool initSupervisorIo(const SupervisorWriteFn callback,
                      void* const callback_context, const int core_id,
                      const unsigned task_priority) {
  if (callback == nullptr || input_queue != nullptr || egress_queue != nullptr ||
      supervisor_task != nullptr) {
    return false;
  }

  input_queue = xQueueCreate(kSupervisorQueueDepth, sizeof(SupervisorInputEvent));
  egress_queue = xQueueCreate(kRuntimeEgressQueueDepth, sizeof(RuntimeEgressRecord));
  if (input_queue == nullptr || egress_queue == nullptr) {
    return false;
  }

  write_fn = callback;
  write_context = callback_context;
  return xTaskCreatePinnedToCore(supervisorIoTask, "triwhirl_supervisor", 4096,
                                 nullptr,
                                 static_cast<UBaseType_t>(task_priority),
                                 &supervisor_task, core_id) == pdPASS;
}

bool tryReceiveSupervisorInput(SupervisorInputEvent* const event) {
  return event != nullptr && input_queue != nullptr &&
         xQueueReceive(input_queue, event, 0) == pdTRUE;
}

bool publishRuntimeReply(const RuntimeReply& reply) {
  RuntimeEgressRecord record{};
  record.type = RuntimeEgressType::kReply;
  record.payload.reply = reply;
  return publishEgress(record);
}

bool publishRuntimeStateEvent(const RuntimeStateEvent& event) {
  RuntimeEgressRecord record{};
  record.type = RuntimeEgressType::kStateEvent;
  record.payload.state_event = event;
  return publishEgress(record);
}

bool publishRuntimeTelemetry(const RuntimeTelemetryFrame& frame) {
  RuntimeEgressRecord record{};
  record.type = RuntimeEgressType::kTelemetry;
  record.payload.telemetry = frame;
  return publishEgress(record);
}

bool publishRuntimeProfileReport(const RuntimeProfileReport& report) {
  RuntimeEgressRecord record{};
  record.type = RuntimeEgressType::kProfileReport;
  record.payload.profile_report = report;
  return publishEgress(record);
}

std::uint32_t runtimeEgressDroppedCount() {
  return egress_dropped;
}

std::uint32_t runtimeReplyDroppedCount() {
  return egress_dropped;
}

}  // namespace triwhirl::runtime
