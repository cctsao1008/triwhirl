#include "runtime_imu_acquisition.hpp"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace triwhirl::runtime {
namespace {

struct ImuAcquisitionRequest {
  std::uint32_t sequence = 0U;
  std::uint32_t requested_at_us = 0U;
};

QueueHandle_t request_queue = nullptr;
QueueHandle_t result_queue = nullptr;
TaskHandle_t acquisition_task = nullptr;
ImuReadFn imu_read_fn = nullptr;
void* imu_read_context = nullptr;
std::uint32_t request_sequence = 0U;
ImuAcquisitionStats stats{};

void imuAcquisitionTask(void*) {
  ImuAcquisitionRequest request{};
  while (true) {
    if (xQueueReceive(request_queue, &request, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    ImuAcquisitionResult result{};
    result.sequence = request.sequence;
    result.requested_at_us = request.requested_at_us;
    result.started_at_us = static_cast<std::uint32_t>(esp_timer_get_time());
    result.ok = imu_read_fn != nullptr &&
                imu_read_fn(imu_read_context, &result.sample);
    result.completed_at_us = static_cast<std::uint32_t>(esp_timer_get_time());
    xQueueOverwrite(result_queue, &result);
  }
}

}  // namespace

bool initImuAcquisition(const ImuReadFn read_fn, void* const context,
                        const int core_id, const unsigned task_priority) {
  if (read_fn == nullptr || request_queue != nullptr || result_queue != nullptr ||
      acquisition_task != nullptr) {
    return false;
  }

  request_queue = xQueueCreate(1U, sizeof(ImuAcquisitionRequest));
  result_queue = xQueueCreate(1U, sizeof(ImuAcquisitionResult));
  if (request_queue == nullptr || result_queue == nullptr) {
    return false;
  }

  imu_read_fn = read_fn;
  imu_read_context = context;
  return xTaskCreatePinnedToCore(
             imuAcquisitionTask, "triwhirl_imu", 4096, nullptr,
             static_cast<UBaseType_t>(task_priority), &acquisition_task,
             core_id) == pdPASS;
}

bool dispatchImuAcquisition(std::uint32_t* const sequence) {
  if (sequence == nullptr || request_queue == nullptr ||
      acquisition_task == nullptr) {
    ++stats.dispatch_failures;
    return false;
  }

  ImuAcquisitionRequest request{};
  request.sequence = ++request_sequence;
  request.requested_at_us = static_cast<std::uint32_t>(esp_timer_get_time());
  if (xQueueSend(request_queue, &request, 0) != pdTRUE) {
    ++stats.dispatch_failures;
    return false;
  }

  ++stats.requests;
  *sequence = request.sequence;
  return true;
}

bool collectImuAcquisition(const std::uint32_t expected_sequence,
                           const std::uint32_t join_budget_us,
                           ImuAcquisitionResult* const result) {
  if (result == nullptr || result_queue == nullptr) {
    ++stats.join_timeouts;
    return false;
  }

  const std::int64_t deadline_us =
      esp_timer_get_time() + static_cast<std::int64_t>(join_budget_us);
  ImuAcquisitionResult candidate{};

  do {
    while (xQueueReceive(result_queue, &candidate, 0) == pdTRUE) {
      if (candidate.sequence == expected_sequence) {
        if (!candidate.ok) {
          ++stats.read_failures;
        }
        *result = candidate;
        return true;
      }
      ++stats.stale_results;
    }
  } while (esp_timer_get_time() < deadline_us);

  ++stats.join_timeouts;
  return false;
}

ImuAcquisitionStats imuAcquisitionStats() {
  return stats;
}

void resetImuAcquisitionStats() {
  stats = {};
}

}  // namespace triwhirl::runtime
