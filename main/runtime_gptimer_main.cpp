// Hardware-timer release wrapper for the parallel sensor runtime.
//
// FreeRTOS tick-based vTaskDelayUntil() was adequate while the control path
// overran 1 ms, but once stopped-motor execution dropped below 1 ms the measured
// start-to-start period became strongly bimodal. Replace only the periodic wait
// used by the established runtime with a 1 MHz GPTimer alarm that wakes the
// pinned core-1 control task every 1000 us. The timer callback does no control
// work; ESP32 firmware remains the realtime authority and the control task owns
// all state updates.

#include <cstdint>

#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr std::uint32_t kRealtimeReleasePeriodUs = 1000U;

gptimer_handle_t realtime_release_timer = nullptr;
TaskHandle_t realtime_release_task = nullptr;
bool realtime_release_init_failed = false;

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

  gptimer_alarm_config_t alarm{};
  alarm.alarm_count = kRealtimeReleasePeriodUs;
  alarm.reload_count = 0U;
  alarm.flags.auto_reload_on_alarm = true;
  if (gptimer_set_alarm_action(realtime_release_timer, &alarm) != ESP_OK ||
      gptimer_enable(realtime_release_timer) != ESP_OK) {
    gptimer_del_timer(realtime_release_timer);
    realtime_release_timer = nullptr;
    realtime_release_init_failed = true;
    return false;
  }

  if (gptimer_start(realtime_release_timer) != ESP_OK) {
    gptimer_disable(realtime_release_timer);
    gptimer_del_timer(realtime_release_timer);
    realtime_release_timer = nullptr;
    realtime_release_init_failed = true;
    return false;
  }
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

  // pdTRUE clears the notification count. If an execution overrun spans more
  // than one 1 kHz alarm, all accumulated releases are coalesced into one next
  // iteration instead of creating a catch-up burst of sub-millisecond loops.
  (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
}

}  // namespace

// All FreeRTOS/task headers are already included above, so this replacement
// affects only runtime call sites rather than the FreeRTOS API declarations.
#define vTaskDelayUntil triwhirlRealtimeDelayUntil
#include "runtime_parallel_main.cpp"
#undef vTaskDelayUntil

static_assert(kRealtimeReleasePeriodUs == kControlPeriodUs,
              "GPTimer release period must match control period");
