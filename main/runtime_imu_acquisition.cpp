#include "runtime_imu_acquisition.hpp"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "triwhirl/board.hpp"

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
bool drdy_irq_enabled = false;
bool drdy_probe_only = false;
int drdy_gpio = -1;

void incrementStat(std::uint64_t ImuAcquisitionStats::* const field) {
  portENTER_CRITICAL(&stats_mux);
  ++(stats.*field);
  portEXIT_CRITICAL(&stats_mux);
}

void mpuDataReadyIsr(void*) {
  portENTER_CRITICAL_ISR(&stats_mux);
  ++stats.drdy_edges;
  portEXIT_CRITICAL_ISR(&stats_mux);

  // An unverified GPIO is observation-only: never let a guessed route become a
  // scheduling authority. A verified route may wake the worker directly later.
  if (drdy_probe_only) {
    return;
  }

  BaseType_t task_woken = pdFALSE;
  if (acquisition_task != nullptr) {
    vTaskNotifyGiveFromISR(acquisition_task, &task_woken);
  }
  if (task_woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

bool initDataReadyInput() {
  if (drdy_gpio >= 0) {
    return drdy_irq_enabled;
  }

  if (triwhirl::board::kMpu6050IntRoutingVerified &&
      triwhirl::board::kMpu6050IntGpio >= 0) {
    drdy_gpio = triwhirl::board::kMpu6050IntGpio;
    drdy_probe_only = false;
  } else if (triwhirl::board::kMpu6050IntProbeGpio >= 0) {
    drdy_gpio = triwhirl::board::kMpu6050IntProbeGpio;
    drdy_probe_only = true;
  } else {
    return false;
  }

  gpio_config_t config{};
  config.pin_bit_mask = 1ULL << static_cast<unsigned>(drdy_gpio);
  config.mode = GPIO_MODE_INPUT;
  // Do not bias an unknown production-board net while probing.
  config.pull_up_en = GPIO_PULLUP_DISABLE;
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.intr_type = GPIO_INTR_POSEDGE;
  if (gpio_config(&config) != ESP_OK) {
    drdy_gpio = -1;
    return false;
  }

  const esp_err_t install = gpio_install_isr_service(0);
  if (install != ESP_OK && install != ESP_ERR_INVALID_STATE) {
    drdy_gpio = -1;
    return false;
  }
  if (gpio_isr_handler_add(static_cast<gpio_num_t>(drdy_gpio),
                           mpuDataReadyIsr, nullptr) != ESP_OK) {
    drdy_gpio = -1;
    return false;
  }
  drdy_irq_enabled = true;
  return true;
}

void imuAcquisitionTask(void*) {
  ImuAcquisitionRequest request{};
  while (true) {
    if (xQueueReceive(request_queue, &request, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    if (drdy_irq_enabled && !drdy_probe_only &&
        ulTaskNotifyTake(pdTRUE, 0) > 0U) {
      incrementStat(&ImuAcquisitionStats::drdy_consumed);
    } else {
      incrementStat(&ImuAcquisitionStats::drdy_fallback_reads);
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
  if (xTaskCreatePinnedToCore(
          imuAcquisitionTask, "triwhirl_imu", 4096, nullptr,
          static_cast<UBaseType_t>(task_priority), &acquisition_task,
          core_id) != pdPASS) {
    acquisition_task = nullptr;
    return false;
  }

  drdy_irq_enabled = initDataReadyInput();
  return true;
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

bool tryCollectImuAcquisition(const std::uint32_t expected_sequence,
                              ImuAcquisitionResult* const result) {
  if (result == nullptr || result_queue == nullptr) {
    return false;
  }

  ImuAcquisitionResult candidate{};
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
  return false;
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
  do {
    if (tryCollectImuAcquisition(expected_sequence, result)) {
      return true;
    }
  } while (esp_timer_get_time() < deadline_us);

  incrementStat(&ImuAcquisitionStats::join_timeouts);
  return false;
}

ImuAcquisitionStats imuAcquisitionStats() {
  // Keep the passive MPU_INT hypothesis observable even when MPU6050 register
  // initialization fails. This probe is deliberately non-authoritative: with
  // an unverified route the ISR only counts rising edges and never wakes the
  // acquisition worker or influences Balance scheduling.
  if (drdy_gpio < 0) {
    initDataReadyInput();
  }

  portENTER_CRITICAL(&stats_mux);
  ImuAcquisitionStats copy = stats;
  portEXIT_CRITICAL(&stats_mux);
  copy.drdy_gpio = drdy_gpio;
  copy.drdy_probe_only = drdy_probe_only;
  return copy;
}

void resetImuAcquisitionStats() {
  portENTER_CRITICAL(&stats_mux);
  stats = {};
  portEXIT_CRITICAL(&stats_mux);
}

}  // namespace triwhirl::runtime
