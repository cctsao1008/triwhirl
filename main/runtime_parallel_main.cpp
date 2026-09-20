// Parallel sensor-acquisition wrapper for the established runtime.
//
// Keep runtime_main.cpp as the single source of the supervisor/swing runtime,
// but substitute the control task created by app_main. The replacement task
// starts the AS5600 transaction on core 0 while core 1 performs the blocking
// MPU6050 transaction and attitude update. The two sensors already live on
// independent ESP32 I2C controllers, so their bus time can overlap without
// moving realtime control authority off the ESP32.

#include <cstdint>
#include <cstring>
#include <limits>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace {

void triwhirlParallelControlTask(void*);

BaseType_t triwhirlCreatePinnedTaskIntercept(
    TaskFunction_t task_code,
    const char* const name,
    const std::uint32_t stack_depth,
    void* const parameters,
    const UBaseType_t priority,
    TaskHandle_t* const created_task,
    const BaseType_t core_id) {
  if (name != nullptr && std::strcmp(name, "triwhirl_control") == 0) {
    task_code = triwhirlParallelControlTask;
  }
  return xTaskCreatePinnedToCore(task_code, name, stack_depth, parameters,
                                 priority, created_task, core_id);
}

}  // namespace

// runtime_main.cpp creates several tasks. Intercept only the task named
// "triwhirl_control" and leave UART/event tasks unchanged.
#define xTaskCreatePinnedToCore triwhirlCreatePinnedTaskIntercept
#include "runtime_main.cpp"
#undef xTaskCreatePinnedToCore

namespace {

struct EncoderAcquisitionRequest {
  std::uint32_t sequence = 0U;
};

struct EncoderAcquisitionResult {
  std::uint32_t sequence = 0U;
  std::uint16_t raw_count = 0U;
  bool ok = false;
};

constexpr std::uint32_t kEncoderJoinBudgetUs = 100U;

QueueHandle_t encoder_request_queue = nullptr;
QueueHandle_t encoder_result_queue = nullptr;
TaskHandle_t encoder_acquisition_task = nullptr;
std::uint32_t encoder_request_sequence = 0U;

std::uint64_t encoder_parallel_requests = 0U;
std::uint64_t encoder_parallel_completions = 0U;
std::uint64_t encoder_parallel_dispatch_failures = 0U;
std::uint64_t encoder_parallel_stale_results = 0U;
std::uint64_t encoder_parallel_join_timeouts = 0U;

void encoderAcquisitionTask(void*) {
  EncoderAcquisitionRequest request{};
  while (true) {
    if (xQueueReceive(encoder_request_queue, &request, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    EncoderAcquisitionResult result{};
    result.sequence = request.sequence;
    result.ok = encoder.readRawAngle(&result.raw_count);

    // The result queue has length one. Keep the newest completed transaction;
    // the sequence number lets the control task reject a stale completion if
    // core 0 was ever delayed long enough to cross a control iteration.
    xQueueOverwrite(encoder_result_queue, &result);
  }
}

bool initParallelEncoderAcquisition() {
  encoder_request_queue = xQueueCreate(1U, sizeof(EncoderAcquisitionRequest));
  encoder_result_queue = xQueueCreate(1U, sizeof(EncoderAcquisitionResult));
  if (encoder_request_queue == nullptr || encoder_result_queue == nullptr) {
    return false;
  }

  return xTaskCreatePinnedToCore(
             encoderAcquisitionTask, "triwhirl_encoder", 4096, nullptr,
             configMAX_PRIORITIES - 2, &encoder_acquisition_task, 0) == pdPASS;
}

bool dispatchEncoderAcquisition(std::uint32_t* const sequence) {
  if (sequence == nullptr || encoder_request_queue == nullptr ||
      encoder_acquisition_task == nullptr) {
    return false;
  }

  EncoderAcquisitionRequest request{};
  request.sequence = ++encoder_request_sequence;
  if (xQueueSend(encoder_request_queue, &request, 0) != pdTRUE) {
    ++encoder_parallel_dispatch_failures;
    return false;
  }

  ++encoder_parallel_requests;
  *sequence = request.sequence;
  return true;
}

bool commitEncoderResult(const EncoderAcquisitionResult& result,
                         const std::uint32_t sample_time_us) {
  if (!result.ok) {
    encoder_sample_valid = false;
    ++encoder_read_errors;
    return false;
  }

  wheel_state = wheel_kinematics.update(result.raw_count, sample_time_us);
  encoder_sample_valid = true;
  ++encoder_parallel_completions;
  return true;
}

bool collectEncoderAcquisition(const std::uint32_t expected_sequence,
                               const std::uint32_t sample_time_us) {
  if (encoder_result_queue == nullptr) {
    encoder_sample_valid = false;
    ++encoder_read_errors;
    return false;
  }

  const std::int64_t deadline_us =
      esp_timer_get_time() + static_cast<std::int64_t>(kEncoderJoinBudgetUs);
  EncoderAcquisitionResult result{};

  do {
    while (xQueueReceive(encoder_result_queue, &result, 0) == pdTRUE) {
      if (result.sequence == expected_sequence) {
        return commitEncoderResult(result, sample_time_us);
      }
      ++encoder_parallel_stale_results;
    }
  } while (esp_timer_get_time() < deadline_us);

  ++encoder_parallel_join_timeouts;
  encoder_sample_valid = false;
  ++encoder_read_errors;
  return false;
}

void triwhirlParallelControlTask(void*) {
  if (!initParallelEncoderAcquisition()) {
    safety_latch.trip(SafetyFault::kStartup);
    stopMotor();
    consoleWrite("FATAL fault=startup parallel encoder task creation failed\r\n");
    vTaskDelete(nullptr);
    return;
  }

  TickType_t last_wake = xTaskGetTickCount();
  while (true) {
    const std::int64_t start_us = esp_timer_get_time();
    const std::uint32_t loop_us = static_cast<std::uint32_t>(start_us);
    const bool profile = runtime_timing_profile.enabled;

    // Kick AS5600 on core 0, then immediately spend core-1 wall time on the
    // MPU6050 transaction and attitude estimator. The AS5600 result normally
    // completes hundreds of microseconds before the MPU path finishes.
    std::uint32_t encoder_sequence = 0U;
    const bool encoder_dispatched =
        dispatchEncoderAcquisition(&encoder_sequence);
    const std::int64_t after_encoder_kick_us = esp_timer_get_time();
    if (profile) {
      // In the parallel runtime this stage is dispatch overhead, not the I2C
      // transfer itself. encoder_i2c_raw below remains the actual bus timing.
      recordRuntimeTimingStage(RuntimeTimingStage::kEncoder, start_us,
                               after_encoder_kick_us);
    }

    const std::int64_t imu_begin_us = after_encoder_kick_us;
    updateImu(loop_us);
    const std::int64_t imu_end_us = esp_timer_get_time();
    if (profile) {
      recordRuntimeTimingStage(RuntimeTimingStage::kImuAttitude, imu_begin_us,
                               imu_end_us);
    }

    if (encoder_dispatched) {
      collectEncoderAcquisition(encoder_sequence, loop_us);
    } else {
      encoder_sample_valid = false;
      ++encoder_read_errors;
    }

    std::int64_t stage_us = esp_timer_get_time();

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
    if (profile) {
      const std::int64_t now = esp_timer_get_time();
      recordRuntimeTimingStage(RuntimeTimingStage::kLog, stage_us, now);
      stage_us = now;
    }

    if ((loop_us - last_supervisor_console_poll_us) >=
        kSupervisorConsolePollPeriodUs) {
      last_supervisor_console_poll_us = loop_us;
      pollSupervisorConsole();
    }
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
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
  }
}

}  // namespace
