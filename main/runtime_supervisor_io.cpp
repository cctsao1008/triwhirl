#include "runtime_supervisor_io.hpp"

#include <cstdio>
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

struct CommandInputState {
  char line[kSupervisorCommandBytes]{};
  std::size_t length = 0U;
};

QueueHandle_t input_queue = nullptr;
TaskHandle_t supervisor_task = nullptr;
SupervisorWriteFn write_fn = nullptr;
void* write_context = nullptr;
CommandInputState uart_input{};
CommandInputState ble_input{};

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
  // Command traffic is supervisory. If the bounded mailbox is full, reject the
  // newest event rather than ever blocking this task or the realtime consumer.
  if (xQueueSend(input_queue, &event, 0) != pdTRUE) {
    writeText("ERR command mailbox full\r\n");
    return false;
  }
  return true;
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

  SupervisorInputEvent event{};
  event.type = SupervisorInputEventType::kReadOnlyHandled;
  publishEvent(event);
  return true;
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
        if (!handleReadOnlySnapshotCommand(state.line)) {
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
    const int uart_received =
        uart_read_bytes(UART_NUM_0, input, sizeof(input), 0);
    if (uart_received > 0) {
      consumeBytes(input, static_cast<std::size_t>(uart_received), uart_input);
    }

    const std::size_t ble_received = triwhirl::ble::read(input, sizeof(input));
    if (ble_received > 0U) {
      consumeBytes(input, ble_received, ble_input);
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
