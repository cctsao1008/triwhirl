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

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "realtime_release.hpp"

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

struct I2cBusCreateContext {
  const i2c_master_bus_config_t* config = nullptr;
  i2c_master_bus_handle_t* output = nullptr;
  SemaphoreHandle_t done = nullptr;
  esp_err_t result = ESP_FAIL;
};

void createI2cBusPinnedTask(void* opaque) {
  auto* context = static_cast<I2cBusCreateContext*>(opaque);
  if (context != nullptr && context->config != nullptr &&
      context->output != nullptr) {
    // This call intentionally refers to the real ESP-IDF API. The preprocessor
    // interception below is not active while this helper is compiled.
    context->result = i2c_new_master_bus(context->config, context->output);
  }
  if (context != nullptr && context->done != nullptr) {
    xSemaphoreGive(context->done);
  }
  vTaskDelete(nullptr);
}

esp_err_t triwhirlI2cNewMasterBusIntercept(
    const i2c_master_bus_config_t* config,
    i2c_master_bus_handle_t* output) {
  if (config == nullptr || output == nullptr) {
    return ESP_ERR_INVALID_ARG;
  }

  // ESP-IDF external peripheral interrupts are allocated on the core which
  // creates them. Keep AS5600/I2C0 on the encoder worker's core 0 and
  // MPU6050/I2C1 on the control task's core 1 so the independent controllers
  // do not both depend on core-0 ISR service while running in parallel.
  const BaseType_t target_core =
      config->i2c_port == I2C_NUM_1 ? 1 : 0;
  if (xPortGetCoreID() == target_core) {
    return i2c_new_master_bus(config, output);
  }

  I2cBusCreateContext context{};
  context.config = config;
  context.output = output;
  context.done = xSemaphoreCreateBinary();
  if (context.done == nullptr) {
    return ESP_ERR_NO_MEM;
  }

  const BaseType_t created = xTaskCreatePinnedToCore(
      createI2cBusPinnedTask, "triwhirl_i2c_init", 4096, &context,
      configMAX_PRIORITIES - 1, nullptr, target_core);
  if (created != pdPASS) {
    vSemaphoreDelete(context.done);
    return ESP_FAIL;
  }

  xSemaphoreTake(context.done, portMAX_DELAY);
  vSemaphoreDelete(context.done);
  return context.result;
}

}  // namespace

// runtime_main.cpp creates several tasks and both I2C buses. Intercept only the
// task named "triwhirl_control" and bind each I2C bus allocation to the core
// which services that sensor in the parallel runtime.
#define xTaskCreatePinnedToCore triwhirlCreatePinnedTaskIntercept
#define i2c_new_master_bus triwhirlI2cNewMasterBusIntercept
#include "runtime_main.cpp"
#undef i2c_new_master_bus
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

struct LocalTimingStats {
  std::uint64_t count = 0U;
  std::uint64_t total_us = 0U;
  std::uint32_t min_us = 0U;
  std::uint32_t max_us = 0U;
};

struct ParallelPeriodHistogram {
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

QueueHandle_t encoder_request_queue = nullptr;
QueueHandle_t encoder_result_queue = nullptr;
TaskHandle_t encoder_acquisition_task = nullptr;
std::uint32_t encoder_request_sequence = 0U;

std::uint64_t encoder_parallel_requests = 0U;
std::uint64_t encoder_parallel_completions = 0U;
std::uint64_t encoder_parallel_dispatch_failures = 0U;
std::uint64_t encoder_parallel_read_failures = 0U;
std::uint64_t encoder_parallel_stale_results = 0U;
std::uint64_t encoder_parallel_join_timeouts = 0U;
std::uint32_t encoder_parallel_consecutive_misses = 0U;
std::uint32_t encoder_parallel_max_consecutive_misses = 0U;

LocalTimingStats attitude_math_timing{};
ParallelPeriodHistogram parallel_period_histogram{};
std::int64_t parallel_previous_start_us = 0;
bool parallel_profile_active = false;

void resetParallelProfileStats() {
  encoder_parallel_requests = 0U;
  encoder_parallel_completions = 0U;
  encoder_parallel_dispatch_failures = 0U;
  encoder_parallel_read_failures = 0U;
  encoder_parallel_stale_results = 0U;
  encoder_parallel_join_timeouts = 0U;
  encoder_parallel_consecutive_misses = 0U;
  encoder_parallel_max_consecutive_misses = 0U;
  attitude_math_timing = {};
  parallel_period_histogram = {};
  parallel_previous_start_us = 0;
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

void recordParallelPeriod(const std::int64_t start_us) {
  if (parallel_previous_start_us == 0 || start_us <= parallel_previous_start_us) {
    parallel_previous_start_us = start_us;
    return;
  }
  const std::uint32_t period_us =
      static_cast<std::uint32_t>(start_us - parallel_previous_start_us);
  parallel_previous_start_us = start_us;
  if (period_us < 900U) {
    ++parallel_period_histogram.lt_900;
  } else if (period_us < 950U) {
    ++parallel_period_histogram.us_900_949;
  } else if (period_us < 1000U) {
    ++parallel_period_histogram.us_950_999;
  } else if (period_us < 1050U) {
    ++parallel_period_histogram.us_1000_1049;
  } else if (period_us < 1100U) {
    ++parallel_period_histogram.us_1050_1099;
  } else if (period_us < 1250U) {
    ++parallel_period_histogram.us_1100_1249;
  } else if (period_us < 1500U) {
    ++parallel_period_histogram.us_1250_1499;
  } else {
    ++parallel_period_histogram.ge_1500;
  }
}

void printParallelProfileSummary() {
  const double attitude_mean_us =
      attitude_math_timing.count > 0U
          ? static_cast<double>(attitude_math_timing.total_us) /
                static_cast<double>(attitude_math_timing.count)
          : 0.0;
  consolePrintf(
      "parallel_profile,requests=%llu,completions=%llu,dispatch_failures=%llu,read_failures=%llu,stale_results=%llu,join_timeouts=%llu,max_consecutive_misses=%lu,attitude_count=%llu,attitude_mean_us=%.3f,attitude_min_us=%lu,attitude_max_us=%lu,period_lt900=%llu,period_900_949=%llu,period_950_999=%llu,period_1000_1049=%llu,period_1050_1099=%llu,period_1100_1249=%llu,period_1250_1499=%llu,period_ge1500=%llu\r\n",
      static_cast<unsigned long long>(encoder_parallel_requests),
      static_cast<unsigned long long>(encoder_parallel_completions),
      static_cast<unsigned long long>(encoder_parallel_dispatch_failures),
      static_cast<unsigned long long>(encoder_parallel_read_failures),
      static_cast<unsigned long long>(encoder_parallel_stale_results),
      static_cast<unsigned long long>(encoder_parallel_join_timeouts),
      static_cast<unsigned long>(encoder_parallel_max_consecutive_misses),
      static_cast<unsigned long long>(attitude_math_timing.count),
      attitude_mean_us,
      static_cast<unsigned long>(attitude_math_timing.min_us),
      static_cast<unsigned long>(attitude_math_timing.max_us),
      static_cast<unsigned long long>(parallel_period_histogram.lt_900),
      static_cast<unsigned long long>(parallel_period_histogram.us_900_949),
      static_cast<unsigned long long>(parallel_period_histogram.us_950_999),
      static_cast<unsigned long long>(parallel_period_histogram.us_1000_1049),
      static_cast<unsigned long long>(parallel_period_histogram.us_1050_1099),
      static_cast<unsigned long long>(parallel_period_histogram.us_1100_1249),
      static_cast<unsigned long long>(parallel_period_histogram.us_1250_1499),
      static_cast<unsigned long long>(parallel_period_histogram.ge_1500));
}

void noteEncoderMiss() {
  ++encoder_read_errors;
  ++encoder_parallel_consecutive_misses;
  if (encoder_parallel_consecutive_misses >
      encoder_parallel_max_consecutive_misses) {
    encoder_parallel_max_consecutive_misses =
        encoder_parallel_consecutive_misses;
  }

  // A single scheduling miss keeps the last coherent wheel state for one
  // control period. The next fresh sample naturally spans the longer dt in
  // WheelKinematics. Two consecutive misses invalidate the encoder and let the
  // existing motor safety path stop FOC instead of reacting to one transient.
  if (encoder_parallel_consecutive_misses >= kEncoderConsecutiveMissLimit) {
    encoder_sample_valid = false;
  }
}

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

  // The worker blocks on its queue or I2C transaction almost all the time, so
  // give dispatch latency precedence over lower-priority BLE/logger work on
  // core 0 without consuming CPU while idle.
  return xTaskCreatePinnedToCore(
             encoderAcquisitionTask, "triwhirl_encoder", 4096, nullptr,
             configMAX_PRIORITIES - 1, &encoder_acquisition_task, 0) == pdPASS;
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
    ++encoder_parallel_read_failures;
    noteEncoderMiss();
    return false;
  }

  wheel_state = wheel_kinematics.update(result.raw_count, sample_time_us);
  encoder_sample_valid = true;
  encoder_parallel_consecutive_misses = 0U;
  ++encoder_parallel_completions;
  return true;
}

bool collectEncoderAcquisition(const std::uint32_t expected_sequence,
                               const std::uint32_t sample_time_us) {
  if (encoder_result_queue == nullptr) {
    noteEncoderMiss();
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
  noteEncoderMiss();
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

    if (profile && !parallel_profile_active) {
      resetParallelProfileStats();
      parallel_profile_active = true;
    } else if (!profile && parallel_profile_active) {
      parallel_profile_active = false;
      printParallelProfileSummary();
    }
    if (profile) {
      recordParallelPeriod(start_us);
    }

    // Kick AS5600 on core 0, then immediately spend core-1 wall time on the
    // MPU6050 transaction and attitude estimator. The AS5600 result normally
    // completes hundreds of microseconds before the MPU path finishes.
    std::uint32_t encoder_sequence = 0U;
    const bool encoder_dispatched =
        dispatchEncoderAcquisition(&encoder_sequence);

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

    // Reuse the existing "encoder" profiler stage for only the post-MPU join
    // overhead. The actual AS5600 bus duration remains independently reported
    // as encoder_i2c_raw by the driver profiler on core 0.
    const std::int64_t encoder_join_begin_us = imu_end_us;
    if (encoder_dispatched) {
      collectEncoderAcquisition(encoder_sequence, loop_us);
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
    triwhirl::runtime::waitForRealtimeRelease(&last_wake, pdMS_TO_TICKS(1));
  }
}

}  // namespace
