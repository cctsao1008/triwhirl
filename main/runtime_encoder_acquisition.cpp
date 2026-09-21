#include "runtime_encoder_acquisition.hpp"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace triwhirl::runtime {
namespace {

struct EncoderAcquisitionRequest {
  std::uint32_t sequence = 0U;
  std::uint32_t requested_at_us = 0U;
};

QueueHandle_t request_queue = nullptr;
QueueHandle_t result_queue = nullptr;
TaskHandle_t acquisition_task = nullptr;
EncoderReadFn encoder_read_fn = nullptr;
void* encoder_read_context = nullptr;
std::uint32_t request_sequence = 0U;
portMUX_TYPE stats_mux = portMUX_INITIALIZER_UNLOCKED;
EncoderAcquisitionStats stats{};

void incrementStat(std::uint64_t EncoderAcquisitionStats::* const field) {
  portENTER_CRITICAL(&stats_mux);
  ++(stats.*field);
  portEXIT_CRITICAL(&stats_mux);
}

TickType_t waitTicksForBudgetUs(const std::uint32_t budget_us) {
  if (budget_us == 0U) {
    return 0;
  }
  const std::uint32_t wait_ms = (budget_us + 999U) / 1000U;
  TickType_t ticks = pdMS_TO_TICKS(wait_ms);
  if (ticks == 0U) {
    ticks = 1U;
  }
  return ticks;
}

void encoderAcquisitionTask(void*) {
  EncoderAcquisitionRequest request{};
  while (true) {
    if (xQueueReceive(request_queue, &request, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    EncoderAcquisitionResult result{};
    result.sequence = request.sequence;
    result.requested_at_us = request.requested_at_us;
    result.started_at_us = static_cast<std::uint32_t>(esp_timer_get_time());
    result.ok = encoder_read_fn != nullptr &&
                encoder_read_fn(encoder_read_context, &result.raw_count);
    result.completed_at_us = static_cast<std::uint32_t>(esp_timer_get_time());
    xQueueOverwrite(result_queue, &result);
  }
}

}  // namespace

bool initEncoderAcquisition(const EncoderReadFn read_fn, void* const context,
                            const int core_id,
                            const unsigned task_priority) {
  if (read_fn == nullptr || request_queue != nullptr || result_queue != nullptr ||
      acquisition_task != nullptr) {
    return false;
  }

  request_queue = xQueueCreate(1U, sizeof(EncoderAcquisitionRequest));
  result_queue = xQueueCreate(1U, sizeof(EncoderAcquisitionResult));
  if (request_queue == nullptr || result_queue == nullptr) {
    return false;
  }

  encoder_read_fn = read_fn;
  encoder_read_context = context;
  return xTaskCreatePinnedToCore(
             encoderAcquisitionTask, "triwhirl_encoder", 4096, nullptr,
             static_cast<UBaseType_t>(task_priority), &acquisition_task,
             core_id) == pdPASS;
}

bool dispatchEncoderAcquisition(std::uint32_t* const sequence) {
  if (sequence == nullptr || request_queue == nullptr ||
      acquisition_task == nullptr) {
    incrementStat(&EncoderAcquisitionStats::dispatch_failures);
    return false;
  }

  EncoderAcquisitionRequest request{};
  request.sequence = ++request_sequence;
  request.requested_at_us = static_cast<std::uint32_t>(esp_timer_get_time());
  if (xQueueSend(request_queue, &request, 0) != pdTRUE) {
    incrementStat(&EncoderAcquisitionStats::dispatch_failures);
    return false;
  }

  incrementStat(&EncoderAcquisitionStats::requests);
  *sequence = request.sequence;
  return true;
}

bool tryCollectEncoderAcquisition(const std::uint32_t expected_sequence,
                                  EncoderAcquisitionResult* const result) {
  if (result == nullptr || result_queue == nullptr) {
    return false;
  }

  EncoderAcquisitionResult candidate{};
  while (xQueueReceive(result_queue, &candidate, 0) == pdTRUE) {
    if (candidate.sequence == expected_sequence) {
      if (!candidate.ok) {
        incrementStat(&EncoderAcquisitionStats::read_failures);
      }
      *result = candidate;
      return true;
    }
    incrementStat(&EncoderAcquisitionStats::stale_results);
  }
  return false;
}

bool collectEncoderAcquisition(const std::uint32_t expected_sequence,
                               const std::uint32_t join_budget_us,
                               EncoderAcquisitionResult* const result) {
  if (result == nullptr || result_queue == nullptr) {
    incrementStat(&EncoderAcquisitionStats::join_timeouts);
    return false;
  }

  if (tryCollectEncoderAcquisition(expected_sequence, result)) {
    return true;
  }
  if (join_budget_us == 0U) {
    incrementStat(&EncoderAcquisitionStats::join_timeouts);
    return false;
  }

  const std::int64_t deadline_us =
      esp_timer_get_time() + static_cast<std::int64_t>(join_budget_us);
  while (esp_timer_get_time() < deadline_us) {
    const std::int64_t now_us = esp_timer_get_time();
    if (now_us >= deadline_us) {
      break;
    }
    const std::uint32_t remaining_us =
        static_cast<std::uint32_t>(deadline_us - now_us);

    EncoderAcquisitionResult candidate{};
    if (xQueueReceive(result_queue, &candidate,
                      waitTicksForBudgetUs(remaining_us)) != pdTRUE) {
      break;
    }
    if (candidate.sequence == expected_sequence) {
      if (!candidate.ok) {
        incrementStat(&EncoderAcquisitionStats::read_failures);
      }
      *result = candidate;
      return true;
    }
    incrementStat(&EncoderAcquisitionStats::stale_results);
  }

  incrementStat(&EncoderAcquisitionStats::join_timeouts);
  return false;
}

EncoderAcquisitionStats encoderAcquisitionStats() {
  portENTER_CRITICAL(&stats_mux);
  const EncoderAcquisitionStats copy = stats;
  portEXIT_CRITICAL(&stats_mux);
  return copy;
}

void resetEncoderAcquisitionStats() {
  portENTER_CRITICAL(&stats_mux);
  stats = {};
  portEXIT_CRITICAL(&stats_mux);
}

}  // namespace triwhirl::runtime
