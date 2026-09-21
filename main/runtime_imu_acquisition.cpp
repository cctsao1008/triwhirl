#include "runtime_imu_acquisition.hpp"

#include <atomic>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
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
SemaphoreHandle_t worker_ready = nullptr;
ImuReadFn imu_read_fn = nullptr;
void* imu_read_context = nullptr;
std::uint32_t request_sequence = 0U;
std::atomic<std::uint32_t> drdy_sequence{0U};
std::atomic<bool> drdy_authoritative{false};
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

void addStat(std::uint64_t ImuAcquisitionStats::* const field,
             const std::uint32_t amount) {
  portENTER_CRITICAL(&stats_mux);
  stats.*field += amount;
  portEXIT_CRITICAL(&stats_mux);
}

bool sequenceReached(const std::uint32_t candidate,
                     const std::uint32_t expected) {
  return static_cast<std::int32_t>(candidate - expected) >= 0;
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

void mpuDataReadyIsr(void*) {
  portENTER_CRITICAL_ISR(&stats_mux);
  ++stats.drdy_edges;
  portEXIT_CRITICAL_ISR(&stats_mux);

  // An unverified GPIO is observation-only. Only an explicitly verified route
  // may become MPU acquisition authority.
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
  if (triwhirl::board::kMpu6050IntRoutingVerified &&
      triwhirl::board::kMpu6050IntGpio >= 0) {
    drdy_gpio = triwhirl::board::kMpu6050IntGpio;
    drdy_probe_only = false;
  } else if (triwhirl::board::kMpu6050IntProbeGpio >= 0) {
    drdy_gpio = triwhirl::board::kMpu6050IntProbeGpio;
    drdy_probe_only = true;
  } else {
    drdy_gpio = -1;
    drdy_probe_only = false;
    return false;
  }

  gpio_config_t config{};
  config.pin_bit_mask = 1ULL << static_cast<unsigned>(drdy_gpio);
  config.mode = GPIO_MODE_INPUT;
  config.pull_up_en = GPIO_PULLUP_DISABLE;
  config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  config.intr_type = GPIO_INTR_POSEDGE;
  if (gpio_config(&config) != ESP_OK) {
    drdy_gpio = -1;
    return false;
  }

  // initImuAcquisition() creates this worker on the I/O core before waiting for
  // worker_ready, so GPIO ISR allocation occurs in the same Core-0 domain as
  // MPU I2C rather than on realtime Core 1.
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
  return true;
}

void runIrqDrivenAcquisition() {
  while (true) {
    const std::uint32_t notifications =
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (notifications == 0U) {
      continue;
    }

    addStat(&ImuAcquisitionStats::drdy_consumed, notifications);

    ImuAcquisitionResult result{};
    result.sequence =
        drdy_sequence.fetch_add(notifications, std::memory_order_acq_rel) +
        notifications;
    result.requested_at_us =
        static_cast<std::uint32_t>(esp_timer_get_time());
    result.started_at_us = result.requested_at_us;
    result.ok = imu_read_fn != nullptr &&
                imu_read_fn(imu_read_context, &result.sample);
    result.completed_at_us = static_cast<std::uint32_t>(esp_timer_get_time());
    if (!result.ok) {
      incrementStat(&ImuAcquisitionStats::read_failures);
    }
    xQueueOverwrite(result_queue, &result);
  }
}

void runRequestDrivenFallback() {
  ImuAcquisitionRequest request{};
  while (true) {
    if (xQueueReceive(request_queue, &request, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    incrementStat(&ImuAcquisitionStats::drdy_fallback_reads);

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

void imuAcquisitionTask(void*) {
  drdy_irq_enabled = initDataReadyInput();
  const bool authoritative =
      drdy_irq_enabled && !drdy_probe_only &&
      triwhirl::board::kMpu6050IntRoutingVerified;
  drdy_authoritative.store(authoritative, std::memory_order_release);

  if (worker_ready != nullptr) {
    xSemaphoreGive(worker_ready);
  }

  if (authoritative) {
    runIrqDrivenAcquisition();
  } else {
    runRequestDrivenFallback();
  }
}

}  // namespace

bool initImuAcquisition(const ImuReadFn read_fn, void* const context,
                        const int core_id, const unsigned task_priority) {
  if (read_fn == nullptr || request_queue != nullptr || result_queue != nullptr ||
      acquisition_task != nullptr || worker_ready != nullptr) {
    return false;
  }

  request_queue = xQueueCreate(1U, sizeof(ImuAcquisitionRequest));
  result_queue = xQueueCreate(1U, sizeof(ImuAcquisitionResult));
  worker_ready = xSemaphoreCreateBinary();
  if (request_queue == nullptr || result_queue == nullptr ||
      worker_ready == nullptr) {
    return false;
  }

  imu_read_fn = read_fn;
  imu_read_context = context;
  if (xTaskCreatePinnedToCore(
          imuAcquisitionTask, "triwhirl_imu", 4096, nullptr,
          static_cast<UBaseType_t>(task_priority), &acquisition_task,
          core_id) != pdPASS) {
    acquisition_task = nullptr;
    vSemaphoreDelete(worker_ready);
    worker_ready = nullptr;
    return false;
  }

  // The worker performs GPIO/ISR setup on its pinned I/O core, then releases
  // startup. This avoids allocating the DATA_RDY interrupt from Core 1.
  xSemaphoreTake(worker_ready, portMAX_DELAY);
  vSemaphoreDelete(worker_ready);
  worker_ready = nullptr;
  return true;
}

bool dispatchImuAcquisition(std::uint32_t* const sequence) {
  if (sequence == nullptr || result_queue == nullptr ||
      acquisition_task == nullptr) {
    incrementStat(&ImuAcquisitionStats::dispatch_failures);
    return false;
  }

  if (drdy_authoritative.load(std::memory_order_acquire)) {
    // Do not trigger an MPU read from the sensor-frame request. The IRQ-owned
    // worker is already waiting for DATA_RDY; the coordinator waits for the
    // first hardware-produced sample newer than this snapshot.
    *sequence = drdy_sequence.load(std::memory_order_acquire) + 1U;
    incrementStat(&ImuAcquisitionStats::requests);
    return true;
  }

  if (request_queue == nullptr) {
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
  if (drdy_authoritative.load(std::memory_order_acquire)) {
    // IRQ mode uses result_queue as a latest-sample mailbox. Do not consume a
    // pre-request sample; the next IRQ overwrites it and satisfies expected.
    if (xQueuePeek(result_queue, &candidate, 0) != pdTRUE ||
        !sequenceReached(candidate.sequence, expected_sequence)) {
      return false;
    }
    *result = candidate;
    return true;
  }

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

  if (tryCollectImuAcquisition(expected_sequence, result)) {
    return true;
  }
  if (join_budget_us == 0U) {
    incrementStat(&ImuAcquisitionStats::join_timeouts);
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

    ImuAcquisitionResult candidate{};
    if (drdy_authoritative.load(std::memory_order_acquire)) {
      // IRQ mode keeps the latest sample in the mailbox rather than consuming
      // it. Block by one scheduler slice, then re-peek the hardware-produced
      // result. This path is inactive until an MPU_INT route is verified.
      vTaskDelay(waitTicksForBudgetUs(remaining_us));
      if (tryCollectImuAcquisition(expected_sequence, result)) {
        return true;
      }
      continue;
    }

    if (xQueueReceive(result_queue, &candidate,
                      waitTicksForBudgetUs(remaining_us)) != pdTRUE) {
      break;
    }
    if (candidate.sequence == expected_sequence) {
      if (!candidate.ok) {
        incrementStat(&ImuAcquisitionStats::read_failures);
      }
      *result = candidate;
      return true;
    }
    incrementStat(&ImuAcquisitionStats::stale_results);
  }

  incrementStat(&ImuAcquisitionStats::join_timeouts);
  return false;
}

ImuAcquisitionStats imuAcquisitionStats() {
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
