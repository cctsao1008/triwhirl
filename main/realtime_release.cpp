#include "realtime_release.hpp"

#include <cstdint>

#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace triwhirl::runtime {
namespace {

constexpr std::uint64_t kRealtimeReleasePeriodUs = 1000U;
constexpr int kRealtimeReleaseInterruptPriority = 3;

gptimer_handle_t realtime_release_timer = nullptr;
TaskHandle_t realtime_release_task = nullptr;
bool realtime_release_init_failed = false;
SemaphoreHandle_t realtime_release_init_done = nullptr;
esp_err_t realtime_release_init_result = ESP_FAIL;

bool IRAM_ATTR realtimeReleaseAlarmCallback(
    gptimer_handle_t,
    const gptimer_alarm_event_data_t*,
    void* user_data) {
  const TaskHandle_t task = static_cast<TaskHandle_t>(user_data);
  if (task == nullptr) {
    return false;
  }

  BaseType_t high_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(task, &high_priority_task_woken);
  return high_priority_task_woken == pdTRUE;
}

void createRealtimeReleaseTimerOnCore0(void*) {
  gptimer_handle_t timer = nullptr;

  gptimer_config_t timer_config{};
  timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timer_config.direction = GPTIMER_COUNT_UP;
  timer_config.resolution_hz = 1000000U;
  timer_config.intr_priority = kRealtimeReleaseInterruptPriority;

  esp_err_t result = gptimer_new_timer(&timer_config, &timer);
  if (result == ESP_OK) {
    gptimer_event_callbacks_t callbacks{};
    callbacks.on_alarm = realtimeReleaseAlarmCallback;
    result = gptimer_register_event_callbacks(timer, &callbacks,
                                              realtime_release_task);
  }

  if (result == ESP_OK) {
    gptimer_alarm_config_t alarm{};
    alarm.alarm_count = kRealtimeReleasePeriodUs;
    alarm.reload_count = 0U;
    alarm.flags.auto_reload_on_alarm = true;
    result = gptimer_set_alarm_action(timer, &alarm);
  }

  if (result == ESP_OK) {
    result = gptimer_enable(timer);
  }
  if (result == ESP_OK) {
    result = gptimer_start(timer);
  }

  if (result != ESP_OK && timer != nullptr) {
    (void)gptimer_stop(timer);
    (void)gptimer_disable(timer);
    (void)gptimer_del_timer(timer);
    timer = nullptr;
  }

  realtime_release_timer = timer;
  realtime_release_init_result = result;
  if (realtime_release_init_done != nullptr) {
    xSemaphoreGive(realtime_release_init_done);
  }
  vTaskDelete(nullptr);
}

bool initRealtimeReleaseTimer() {
  if (realtime_release_timer != nullptr) {
    return true;
  }
  if (realtime_release_init_failed) {
    return false;
  }

  realtime_release_task = xTaskGetCurrentTaskHandle();
  realtime_release_init_done = xSemaphoreCreateBinary();
  if (realtime_release_init_done == nullptr) {
    realtime_release_init_failed = true;
    return false;
  }

  // GPTimer peripheral interrupts are allocated on the core that creates the
  // timer. Create it from a short-lived core-0 task so its periodic ISR never
  // preempts the core-1 MPU6050/control critical path.
  const BaseType_t created = xTaskCreatePinnedToCore(
      createRealtimeReleaseTimerOnCore0, "triwhirl_release_init", 4096, nullptr,
      configMAX_PRIORITIES - 1, nullptr, 0);
  if (created != pdPASS) {
    vSemaphoreDelete(realtime_release_init_done);
    realtime_release_init_done = nullptr;
    realtime_release_init_failed = true;
    return false;
  }

  xSemaphoreTake(realtime_release_init_done, portMAX_DELAY);
  vSemaphoreDelete(realtime_release_init_done);
  realtime_release_init_done = nullptr;

  if (realtime_release_init_result != ESP_OK ||
      realtime_release_timer == nullptr) {
    realtime_release_init_failed = true;
    return false;
  }

  // Initialization itself can take longer than one period. Drop notifications
  // accumulated while waiting for the helper so the first real release begins
  // from a fresh hardware tick.
  (void)ulTaskNotifyTake(pdTRUE, 0);
  return true;
}

}  // namespace

void waitForRealtimeRelease(TickType_t* const previous_wake,
                            const TickType_t fallback_increment) {
  if (!initRealtimeReleaseTimer()) {
    // Preserve the established scheduler as a diagnostic fallback during this
    // behavior-preserving refactor. Balance-mode hardening will later make
    // release-clock availability an explicit admission requirement.
    vTaskDelayUntil(previous_wake, fallback_increment);
    return;
  }

  // If the 1 kHz tick arrived while the iteration was still active, the
  // deadline was missed. Clear all accumulated ticks and wait for the next
  // future hardware release instead of starting an immediate catch-up loop.
  (void)ulTaskNotifyTake(pdTRUE, 0);
  (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

}  // namespace triwhirl::runtime
