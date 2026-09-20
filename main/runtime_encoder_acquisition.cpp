#include "runtime_encoder_acquisition.hpp"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace triwhirl::runtime {
namespace {

struct EncoderAcquisitionRequest {
  std::uint32_t sequence = 0U;
};

QueueHandle_t request_queue = nullptr;
QueueHandle_t result_queue = nullptr;
TaskHandle_t acquisition_task = nullptr;
EncoderReadFn encoder_read_fn = nullptr;
void* encoder_read_context = nullptr;
std::uint32_t request_sequence = 0U;
EncoderAcquisitionStats stats{};

void encoderAcquisitionTask(void*) {
  EncoderAcquisitionRequest request{};
  while (true) {
    if (xQueueReceive(request_queue, &request, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    EncoderAcquisitionResult result{};
    result.sequence = request.sequence;
    result.ok = encoder_read_fn != nullptr &&
                encoder_read_fn(encoder_read_context, &result.raw_count);
    if (!result.ok) {
      ++stats.read_failures;
    }
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
    ++stats.dispatch_failures;
    return false;
  }

  EncoderAcquisitionRequest request{};
  request.sequence = ++request_sequence;
  if (xQueueSend(request_queue, &request, 0) != pdTRUE) {
    ++stats.dispatch_failures;
    return false;
  }

  ++stats.requests;
  *sequence = request.sequence;
  return true;
}

bool collectEncoderAcquisition(const std::uint32_t expected_sequence,
                               const std::uint32_t join_budget_us,
                               EncoderAcquisitionResult* const result) {
  if (result == nullptr || result_queue == nullptr) {
    ++stats.join_timeouts;
    return false;
  }

  const std::int64_t deadline_us =
      esp_timer_get_time() + static_cast<std::int64_t>(join_budget_us);
  EncoderAcquisitionResult candidate{};

  do {
    while (xQueueReceive(result_queue, &candidate, 0) == pdTRUE) {
      if (candidate.sequence == expected_sequence) {
        *result = candidate;
        return true;
      }
      ++stats.stale_results;
    }
  } while (esp_timer_get_time() < deadline_us);

  ++stats.join_timeouts;
  return false;
}

EncoderAcquisitionStats encoderAcquisitionStats() {
  return stats;
}

void resetEncoderAcquisitionStats() {
  stats = {};
}

}  // namespace triwhirl::runtime
