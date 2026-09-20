#include "runtime_release.hpp"

#include <algorithm>

#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace triwhirl::runtime {
namespace {

gptimer_handle_t realtime_release_timer = nullptr;
TaskHandle_t realtime_release_task = nullptr;
bool realtime_release_init_failed = false;
SemaphoreHandle_t realtime_release_init_done = nullptr;
esp_err_t realtime_release_init_result = ESP_FAIL;
RealtimeReleaseStats realtime_release_stats{};

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
  // timer. Create it from a short-lived core-0 task so its periodic ISR stays
  // away from the core-1 MPU6050/control critical path.
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

  // Timer setup can span multiple periods. Start the runtime from a fresh
  // release rather than replaying setup-time notifications.
  (void)ulTaskNotifyTake(pdTRUE, 0);
  return true;
}

void accountReleaseNotifications(const std::uint32_t notifications,
                                 const bool one_is_current_release) {
  if (notifications == 0U) {
    return;
  }

  realtime_release_stats.release_ticks += notifications;

  const std::uint32_t missed =
      one_is_current_release ? notifications - 1U : notifications;
  if (missed > 0U) {
    realtime_release_stats.missed_release_ticks += missed;
    ++realtime_release_stats.skip_events;
  }
  realtime_release_stats.max_notification_backlog =
      std::max(realtime_release_stats.max_notification_backlog, notifications);
}

}  // namespace

bool waitForNextRealtimeRelease() {
  if (!initRealtimeReleaseTimer()) {
    return false;
  }

  ++realtime_release_stats.wait_calls;

  // Drop every release which arrived while the current iteration was still
  // active. The returned task-notification count is direct evidence of how many
  // hardware releases were missed; do not infer that from an execution-time
  // histogram.
  const std::uint32_t pending = ulTaskNotifyTake(pdTRUE, 0);
  accountReleaseNotifications(pending, false);

  // Wait for one future release. If more than one notification is already
  // accumulated when the task resumes, exactly one is the release used for the
  // next iteration and the remainder are additional missed releases.
  const std::uint32_t next = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  accountReleaseNotifications(next, true);
  return next > 0U;
}

RealtimeReleaseStats realtimeReleaseStats() {
  return realtime_release_stats;
}

void resetRealtimeReleaseStats() {
  realtime_release_stats = {};
}

}  // namespace triwhirl::runtime
