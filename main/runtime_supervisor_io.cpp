#include "runtime_supervisor_io.hpp"

#include <cstdio>
#include <cstring>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "runtime_command_parser.hpp"
#include "runtime_snapshot.hpp"
#include "triwhirl/ble_transport.hpp"
#include "triwhirl/safety.hpp"

namespace triwhirl::runtime {
namespace {

constexpr std::uint32_t kSupervisorPollPeriodMs = 5U;
constexpr UBaseType_t kSupervisorQueueDepth = 4U;

struct CommandInputState {
  char line[kSupervisorCommandBytes]{};
  std::size_t length = 0U;
};

QueueHandle_t input_queue = nullptr;
TaskHandle_t supervisor_task = nullptr;
SupervisorWriteFn write_fn = nullptr;
void* write_context = nullptr;
CommandInputState uart_development_input{};
CommandInputState ble_gatt_input{};

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

bool publishEvent(const SupervisorInputEvent& event) {
  if (input_queue == nullptr) {
    return false;
  }
  if (xQueueSend(input_queue, &event, 0) != pdTRUE) {
    writeText("ERR command mailbox full\r\n");
    return false;
  }
  return true;
}

void publishSupervisorHandled() {
  SupervisorInputEvent event{};
  event.type = SupervisorInputEventType::kSupervisorHandled;
  publishEvent(event);
}

void writeRuntimeSnapshotUnavailable() {
  writeText("ERR runtime snapshot unavailable\r\n");
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
    publishSupervisorHandled();
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
    if (length > 0) {
      const std::size_t count = static_cast<std::size_t>(length) < sizeof(buffer)
                                    ? static_cast<std::size_t>(length)
                                    : sizeof(buffer) - 1U;
      writeBytes(buffer, count);
    }
    publishSupervisorHandled();
    return true;
  }

  const bool attitude_status = std::strcmp(line, "attitude") == 0 ||
                               std::strcmp(line, "attitude status") == 0;
  const bool fault_status = std::strcmp(line, "fault") == 0 ||
                            std::strcmp(line, "fault status") == 0;
  const bool timing_status = std::strcmp(line, "timing") == 0 ||
                             std::strcmp(line, "timing status") == 0;
  const bool telemetry_status = std::strcmp(line, "telemetry") == 0;
  if (!attitude_status && !fault_status && !timing_status && !telemetry_status) {
    return false;
  }

  RuntimeSnapshot snapshot{};
  if (!readLatestRuntimeSnapshot(&snapshot)) {
    writeRuntimeSnapshotUnavailable();
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
    if (length > 0) {
      const std::size_t count = static_cast<std::size_t>(length) < sizeof(buffer)
                                    ? static_cast<std::size_t>(length)
                                    : sizeof(buffer) - 1U;
      writeBytes(buffer, count);
    }
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
    if (length > 0) {
      const std::size_t count = static_cast<std::size_t>(length) < sizeof(buffer)
                                    ? static_cast<std::size_t>(length)
                                    : sizeof(buffer) - 1U;
      writeBytes(buffer, count);
    }
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
    if (length > 0) {
      const std::size_t count = static_cast<std::size_t>(length) < sizeof(buffer)
                                    ? static_cast<std::size_t>(length)
                                    : sizeof(buffer) - 1U;
      writeBytes(buffer, count);
    }
  } else {
    writeText(snapshot.telemetry_enabled ? "telemetry=on\r\n"
                                         : "telemetry=off\r\n");
  }

  publishSupervisorHandled();
  return true;
}

bool handleTypedRuntimeCommand(const char* const line) {
  const RuntimeCommandParseResult parsed = parseRuntimeCommand(line);
  switch (parsed.status) {
    case RuntimeCommandParseStatus::kNotMatched:
      return false;
    case RuntimeCommandParseStatus::kUsageError:
      writeText(parsed.error);
      publishSupervisorHandled();
      return true;
    case RuntimeCommandParseStatus::kCommand: {
      SupervisorInputEvent event{};
      event.type = SupervisorInputEventType::kRuntimeCommand;
      event.runtime_command = parsed.command;
      publishEvent(event);
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
          publishSupervisorHandled();
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
    SupervisorInputEvent event{};
    event.type = SupervisorInputEventType::kLineOverflow;
    publishEvent(event);
  }
}

void supervisorIoTask(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  std::uint8_t input[64];

  while (true) {
    // UART0 remains an explicit wired development/service CLI. It is not the
    // product BLE transport and has no realtime authority.
    const int uart_received = uart_read_bytes(UART_NUM_0, input, sizeof(input), 0);
    if (uart_received > 0) {
      consumeBytes(input, static_cast<std::size_t>(uart_received),
                   uart_development_input);
    }

    // ble::read() drains payload bytes previously accepted by the NimBLE GATT
    // RX characteristic write callback. BLE command ingress is GATT, not UART.
    const std::size_t ble_received = triwhirl::ble::read(input, sizeof(input));
    if (ble_received > 0U) {
      consumeBytes(input, ble_received, ble_gatt_input);
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kSupervisorPollPeriodMs));
  }
}

}  // namespace

bool initSupervisorIo(const SupervisorWriteFn callback,
                      void* const callback_context, const int core_id,
                      const unsigned task_priority) {
  if (callback == nullptr || input_queue != nullptr || supervisor_task != nullptr) {
    return false;
  }

  input_queue = xQueueCreate(kSupervisorQueueDepth, sizeof(SupervisorInputEvent));
  if (input_queue == nullptr) {
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

}  // namespace triwhirl::runtime
