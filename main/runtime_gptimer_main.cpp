// Hardware-timer release wrapper for the parallel sensor runtime.
//
// FreeRTOS tick-based vTaskDelayUntil() was adequate while the control path
// overran 1 ms, but once stopped-motor execution dropped below 1 ms the measured
// start-to-start period became strongly bimodal. Use a hardware GPTimer as the
// release clock while keeping the control work in the pinned core-1 task.
//
// The first GPTimer version allocated a periodic interrupt on core 1. Hardware
// profiling showed that the 1 kHz timer ISR itself competed with the MPU6050
// I2C completion path and inflated the control execution time. The following
// one-shot version avoided active-loop interrupts, but it had to call
// gptimer_get_raw_count() and gptimer_set_alarm_action() after every iteration;
// with only a few tens of microseconds of headroom that scheduler bookkeeping
// could itself cross the upcoming boundary and turn an otherwise sub-1-ms loop
// into a skipped 2-ms release.
//
// Final policy used here:
//   * allocate one periodic 1 kHz GPTimer on core 0;
//   * keep the timer ISR minimal: only notify the core-1 control task;
//   * never execute control work in the ISR;
//   * at the end of each control iteration, discard any notification that
//     arrived while the iteration was still active, then wait for the next
//     timer tick. Therefore missed releases are skipped, never caught up.
//
// This removes per-iteration GPTimer API calls from the deadline path and keeps
// the release interrupt away from the MPU6050/control core. Core 0 already owns
// BLE/background work and the AS5600 worker, but GPTimer runs as a short level-3
// interrupt and therefore has bounded precedence over those tasks.

#include <cstdint>

#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

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
    // Stop/disable tolerate the state checks below poorly if the corresponding
    // transition never succeeded, so only delete after best-effort cleanup.
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

  // Initialization itself can take longer than one period. Drop any timer
  // notification accumulated while the control task waited for the helper so
  // the first real release begins from a fresh hardware tick.
  (void)ulTaskNotifyTake(pdTRUE, 0);
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

  // If the 1 kHz tick arrived while this iteration was still active, the
  // deadline was missed. Clear all accumulated ticks and wait for the next
  // future hardware release rather than starting an immediate catch-up loop.
  (void)ulTaskNotifyTake(pdTRUE, 0);
  (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
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
