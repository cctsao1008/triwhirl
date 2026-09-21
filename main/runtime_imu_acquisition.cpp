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
portMUX_TYPE stats_mux = portMUX_INITIALIZER_UNLOCKED;
ImuAcquisitionStats stats{};

void incrementStat(std::uint64_t ImuAcquisitionStats::* const field) {
  portENTER_CRITICAL(&stats_mux);
  ++(stats.*field);
  portEXIT_CRITICAL(&stats_mux);
}

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
    incrementStat(&ImuAcquisitionStats::dispatch_failures);
    return false;
  }

  ImuAcquisitionRequest request{};
  request.sequence = ++request_sequence;
  request.requested_at_us = static_cast<std::uint32_t>(esp_timer_get_time());
  if (xQueueSend(request_queue, &request, 0) != pdTRUE) {
    incrementStat(&ImuAcquisitionStats::dispatch_failures);
    return false;
  }

  incrementStat(&ImuAcquisitionStats::requests);
  *sequence = request.sequence;
  return true;
}

bool collectImuAcquisition(const std::uint32_t expected_sequence,
                           const std::uint32_t join_budget_us,
                           ImuAcquisitionResult* const result) {
  if (result == nullptr || result_queue == nullptr) {
    incrementStat(&ImuAcquisitionStats::join_timeouts);
    return false;
  }

  const std::int64_t deadline_us =
      esp_timer_get_time() + static_cast<std::int64_t>(join_budget_us);
  ImuAcquisitionResult candidate{};

  do {
    while (xQueueReceive(result_queue, &candidate, 0) == pdTRUE) {
      if (candidate.sequence == expected_sequence) {
        if (!candidate.ok) {
          incrementStat(&ImuAcquisitionStats::read_failures);
        }
        *result = candidate;
        return true;
      }
      incrementStat(&ImuAcquisitionStats::stale_results);
    }
  } while (esp_timer_get_time() < deadline_us);

  incrementStat(&ImuAcquisitionStats::join_timeouts);
  return false;
}

ImuAcquisitionStats imuAcquisitionStats() {
  portENTER_CRITICAL(&stats_mux);
  const ImuAcquisitionStats copy = stats;
  portEXIT_CRITICAL(&stats_mux);
  return copy;
}

void resetImuAcquisitionStats() {
  portENTER_CRITICAL(&stats_mux);
  stats = {};
  portEXIT_CRITICAL(&stats_mux);
}

}  // namespace triwhirl::runtime
