#include "runtime_supervisor_io.hpp"

#include <cstdio>
#include <cstring>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "runtime_command_parser.hpp"
#include "runtime_diagnostics.hpp"
#include "runtime_snapshot.hpp"
#include "triwhirl/ble_transport.hpp"
#include "triwhirl/runtime_logger.hpp"
#include "triwhirl/safety.hpp"

namespace triwhirl::runtime {
namespace {

constexpr std::uint32_t kSupervisorPollPeriodMs = 5U;
constexpr UBaseType_t kSupervisorQueueDepth = 4U;
constexpr UBaseType_t kRuntimeReplyQueueDepth = 8U;

struct CommandInputState {
  char line[kSupervisorCommandBytes]{};
  std::size_t length = 0U;
};

QueueHandle_t input_queue = nullptr;
QueueHandle_t reply_queue = nullptr;
TaskHandle_t supervisor_task = nullptr;
SupervisorWriteFn write_fn = nullptr;
void* write_context = nullptr;
CommandInputState uart_development_input{};
CommandInputState ble_gatt_input{};
std::uint32_t reply_dropped = 0U;

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

void formatRuntimeReply(const RuntimeReply& reply) {
  char buffer[256];
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
    case RuntimeReplyCode::kTimingResetOk:
      writeText("OK timing reset\r\n");
      break;
    case RuntimeReplyCode::kTimingProfileResetOk:
      writeText("OK timing profile reset\r\n");
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
  }

  if (reply.prompt_after) {
    writeText("> ");
  }
}

void drainRuntimeReplies() {
  if (reply_queue == nullptr) {
    return;
  }
  RuntimeReply reply{};
  while (xQueueReceive(reply_queue, &reply, 0) == pdTRUE) {
    formatRuntimeReply(reply);
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
    char buffer[640];
    const int length = std::snprintf(
        buffer, sizeof(buffer),
        "imu,ready=%d,sample_ok=%d,who_ok=%d,who=0x%02x,bias_valid=%d,calibrating=%d,ax=%.6f,ay=%.6f,az=%.6f,gx=%.6f,gy=%.6f,gz=%.6f,temp_c=%.3f,bx=%.6f,by=%.6f,bz=%.6f,map=%d:%d:%d:%d:%d:%d,read_errors=%lu\r\n",
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
        static_cast<unsigned long>(snapshot.imu_read_errors));
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
    drainRuntimeReplies();

    const int uart_received = uart_read_bytes(UART_NUM_0, input, sizeof(input), 0);
    if (uart_received > 0) {
      consumeBytes(input, static_cast<std::size_t>(uart_received),
                   uart_development_input);
    }

    const std::size_t ble_received = triwhirl::ble::read(input, sizeof(input));
    if (ble_received > 0U) {
      consumeBytes(input, ble_received, ble_gatt_input);
    }

    drainRuntimeReplies();
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kSupervisorPollPeriodMs));
  }
}

}  // namespace

bool initSupervisorIo(const SupervisorWriteFn callback,
                      void* const callback_context, const int core_id,
                      const unsigned task_priority) {
  if (callback == nullptr || input_queue != nullptr || reply_queue != nullptr ||
      supervisor_task != nullptr) {
    return false;
  }

  input_queue = xQueueCreate(kSupervisorQueueDepth, sizeof(SupervisorInputEvent));
  reply_queue = xQueueCreate(kRuntimeReplyQueueDepth, sizeof(RuntimeReply));
  if (input_queue == nullptr || reply_queue == nullptr) {
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
  if (reply_queue == nullptr || xQueueSend(reply_queue, &reply, 0) != pdTRUE) {
    ++reply_dropped;
    return false;
  }
  return true;
}

std::uint32_t runtimeReplyDroppedCount() {
  return reply_dropped;
}

}  // namespace triwhirl::runtime
