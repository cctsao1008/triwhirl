#include "runtime_supervisor_io.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "runtime_snapshot.hpp"
#include "triwhirl/ble_transport.hpp"
#include "triwhirl/safety.hpp"

namespace triwhirl::runtime {
namespace {

constexpr std::uint32_t kSupervisorPollPeriodMs = 5U;
constexpr UBaseType_t kSupervisorQueueDepth = 4U;
constexpr std::uint32_t kDefaultGyroCalibrationSamples = 500U;

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

bool commandArguments(const char* const line, const char* const command,
                      const char** const arguments) {
  if (line == nullptr || command == nullptr || arguments == nullptr) {
    return false;
  }
  const std::size_t length = std::strlen(command);
  if (std::strncmp(line, command, length) != 0) {
    return false;
  }
  const char next = line[length];
  if (next != '\0' && next != ' ' && next != '\t') {
    return false;
  }
  const char* cursor = line + length;
  while (*cursor == ' ' || *cursor == '\t') {
    ++cursor;
  }
  *arguments = cursor;
  return true;
}

char* nextToken(char** const cursor) {
  if (cursor == nullptr || *cursor == nullptr) {
    return nullptr;
  }
  while (**cursor == ' ' || **cursor == '\t') {
    ++(*cursor);
  }
  if (**cursor == '\0') {
    return nullptr;
  }
  char* token = *cursor;
  while (**cursor != '\0' && **cursor != ' ' && **cursor != '\t') {
    ++(*cursor);
  }
  if (**cursor != '\0') {
    **cursor = '\0';
    ++(*cursor);
  }
  return token;
}

bool handleReadOnlySnapshotCommand(const char* const line) {
  if (line == nullptr) {
    return false;
  }

  const bool attitude_status = std::strcmp(line, "attitude status") == 0;
  const bool fault_status = std::strcmp(line, "fault status") == 0;
  if (!attitude_status && !fault_status) {
    return false;
  }

  RuntimeSnapshot snapshot{};
  if (!readLatestRuntimeSnapshot(&snapshot)) {
    writeText("ERR runtime snapshot unavailable\r\n");
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
  } else {
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
  }

  publishSupervisorHandled();
  return true;
}

bool publishRuntimeCommand(const RuntimeCommand& command) {
  SupervisorInputEvent event{};
  event.type = SupervisorInputEventType::kRuntimeCommand;
  event.runtime_command = command;
  publishEvent(event);
  return true;
}

bool handleTypedRuntimeCommand(const char* const line) {
  if (line == nullptr) {
    return false;
  }

  RuntimeCommand command{};
  if (std::strcmp(line, "motor stop") == 0) {
    command.type = RuntimeCommandType::kMotorStop;
    return publishRuntimeCommand(command);
  }
  if (std::strcmp(line, "stop") == 0) {
    command.type = RuntimeCommandType::kStop;
    return publishRuntimeCommand(command);
  }
  if (std::strcmp(line, "swing abort") == 0) {
    command.type = RuntimeCommandType::kSwingAbort;
    return publishRuntimeCommand(command);
  }
  if (std::strcmp(line, "timing reset") == 0) {
    command.type = RuntimeCommandType::kTimingReset;
    return publishRuntimeCommand(command);
  }
  if (std::strcmp(line, "fault clear") == 0) {
    command.type = RuntimeCommandType::kFaultClear;
    return publishRuntimeCommand(command);
  }
  if (std::strcmp(line, "telemetry on") == 0) {
    command.type = RuntimeCommandType::kTelemetryOn;
    return publishRuntimeCommand(command);
  }
  if (std::strcmp(line, "telemetry off") == 0) {
    command.type = RuntimeCommandType::kTelemetryOff;
    return publishRuntimeCommand(command);
  }

  const char* arguments = nullptr;
  if (commandArguments(line, "motor vq", &arguments)) {
    if (*arguments == '\0') {
      writeText("ERR usage: motor vq <volts>\r\n");
      publishSupervisorHandled();
      return true;
    }
    command.type = RuntimeCommandType::kMotorVq;
    command.payload.motor_vq.volts = std::strtof(arguments, nullptr);
    return publishRuntimeCommand(command);
  }

  if (commandArguments(line, "field", &arguments)) {
    char copy[kSupervisorCommandBytes]{};
    std::snprintf(copy, sizeof(copy), "%s", arguments);
    char* cursor = copy;
    char* hz_token = nextToken(&cursor);
    char* amplitude_token = nextToken(&cursor);
    if (hz_token == nullptr || amplitude_token == nullptr) {
      writeText("ERR usage: field <electrical_hz> <amplitude_v>\r\n");
      publishSupervisorHandled();
      return true;
    }
    command.type = RuntimeCommandType::kField;
    command.payload.field.electrical_hz = std::strtof(hz_token, nullptr);
    command.payload.field.amplitude_v = std::strtof(amplitude_token, nullptr);
    return publishRuntimeCommand(command);
  }

  if (commandArguments(line, "attitude reset", &arguments)) {
    command.type = RuntimeCommandType::kAttitudeReset;
    if (*arguments == '\0') {
      command.payload.attitude_reset.use_accelerometer = true;
    } else {
      command.payload.attitude_reset.use_accelerometer = false;
      command.payload.attitude_reset.angle_rad = std::strtof(arguments, nullptr);
    }
    return publishRuntimeCommand(command);
  }

  if (commandArguments(line, "imu calibrate", &arguments)) {
    command.type = RuntimeCommandType::kImuCalibrate;
    command.payload.imu_calibrate.samples =
        *arguments == '\0'
            ? kDefaultGyroCalibrationSamples
            : static_cast<std::uint32_t>(std::strtoul(arguments, nullptr, 10));
    return publishRuntimeCommand(command);
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
        if (!handleReadOnlySnapshotCommand(state.line) &&
            !handleTypedRuntimeCommand(state.line)) {
          SupervisorInputEvent event{};
          event.type = SupervisorInputEventType::kCommand;
          std::memcpy(event.line, state.line, state.length + 1U);
          publishEvent(event);
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
