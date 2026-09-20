// Hardware-timer release wrapper for the parallel sensor runtime.
//
// FreeRTOS tick-based vTaskDelayUntil() was adequate while the control path
// overran 1 ms, but once stopped-motor execution dropped below 1 ms the measured
// start-to-start period became strongly bimodal. Replace only the periodic wait
// used by the established runtime with a 1 MHz GPTimer alarm that wakes the
// pinned core-1 control task at the next absolute 1 ms release boundary.
//
// Use one-shot alarms rather than an auto-reloading periodic alarm. A periodic
// alarm continues to interrupt core 1 while the control iteration is still
// running and can leave a task notification pending; consuming that pending
// notification at the end of the loop creates catch-up releases and also adds
// interrupt contention to the MPU/I2C critical path. With a one-shot alarm the
// timer is disarmed as soon as it wakes the task, so no GPTimer interrupt can
// fire during the active control iteration. The next alarm is armed only when
// the task reaches the release wait at the end of that iteration.
//
// The timer callback does no control work; ESP32 firmware remains the realtime
// authority and the control task owns all state updates.

#include <cstdint>

#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr std::uint64_t kRealtimeReleasePeriodUs = 1000U;

gptimer_handle_t realtime_release_timer = nullptr;
TaskHandle_t realtime_release_task = nullptr;
bool realtime_release_init_failed = false;
std::uint64_t realtime_next_release_count = 0U;

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

bool initRealtimeReleaseTimer() {
  if (realtime_release_timer != nullptr) {
    return true;
  }
  if (realtime_release_init_failed) {
    return false;
  }

  realtime_release_task = xTaskGetCurrentTaskHandle();

  gptimer_config_t timer_config{};
  timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timer_config.direction = GPTIMER_COUNT_UP;
  timer_config.resolution_hz = 1000000U;
  if (gptimer_new_timer(&timer_config, &realtime_release_timer) != ESP_OK) {
    realtime_release_timer = nullptr;
    realtime_release_init_failed = true;
    return false;
  }

  gptimer_event_callbacks_t callbacks{};
  callbacks.on_alarm = realtimeReleaseAlarmCallback;
  if (gptimer_register_event_callbacks(realtime_release_timer, &callbacks,
                                       realtime_release_task) != ESP_OK) {
    gptimer_del_timer(realtime_release_timer);
    realtime_release_timer = nullptr;
    realtime_release_init_failed = true;
    return false;
  }

  if (gptimer_enable(realtime_release_timer) != ESP_OK ||
      gptimer_start(realtime_release_timer) != ESP_OK) {
    gptimer_disable(realtime_release_timer);
    gptimer_del_timer(realtime_release_timer);
    realtime_release_timer = nullptr;
    realtime_release_init_failed = true;
    return false;
  }

  std::uint64_t now_count = 0U;
  if (gptimer_get_raw_count(realtime_release_timer, &now_count) != ESP_OK) {
    gptimer_stop(realtime_release_timer);
    gptimer_disable(realtime_release_timer);
    gptimer_del_timer(realtime_release_timer);
    realtime_release_timer = nullptr;
    realtime_release_init_failed = true;
    return false;
  }
  realtime_next_release_count = now_count + kRealtimeReleasePeriodUs;
  return true;
}

void triwhirlRealtimeDelayUntil(TickType_t* const previous_wake,
                                const TickType_t increment) {
  if (!initRealtimeReleaseTimer()) {
    // Keep the established scheduler as a safe fallback if GPTimer setup ever
    // fails on a different board/configuration.
    vTaskDelayUntil(previous_wake, increment);
    return;
  }

  std::uint64_t now_count = 0U;
  if (gptimer_get_raw_count(realtime_release_timer, &now_count) != ESP_OK) {
    vTaskDelayUntil(previous_wake, increment);
    return;
  }

  // If this iteration ran past its intended release boundary, start the next
  // iteration immediately and re-phase to the first future 1 ms boundary. This
  // avoids a burst of accumulated notifications while preserving the absolute
  // periodic schedule for the following iteration.
  if (now_count >= realtime_next_release_count) {
    do {
      realtime_next_release_count += kRealtimeReleasePeriodUs;
    } while (now_count >= realtime_next_release_count);
    return;
  }

  // A one-shot alarm is armed only while the task is waiting. It fires once,
  // disables itself naturally, and therefore cannot preempt the active control
  // iteration that follows.
  gptimer_alarm_config_t alarm{};
  alarm.alarm_count = realtime_next_release_count;
  alarm.reload_count = 0U;
  alarm.flags.auto_reload_on_alarm = false;

  // The one-shot design should leave no stale notification. Drain defensively
  // before arming so an earlier fallback/error path cannot create an immediate
  // false release.
  (void)ulTaskNotifyTake(pdTRUE, 0);

  if (gptimer_set_alarm_action(realtime_release_timer, &alarm) != ESP_OK) {
    vTaskDelayUntil(previous_wake, increment);
    return;
  }

  (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  realtime_next_release_count += kRealtimeReleasePeriodUs;
}

}  // namespace

// All FreeRTOS/task headers are already included above, so this replacement
// affects only runtime call sites rather than the FreeRTOS API declarations.
#undef vTaskDelayUntil
#define vTaskDelayUntil triwhirlRealtimeDelayUntil
#include "runtime_parallel_main.cpp"
#undef vTaskDelayUntil

static_assert(kRealtimeReleasePeriodUs == kControlPeriodUs,
              "GPTimer release period must match control period");
