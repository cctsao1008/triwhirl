#include "runtime_motor_task.hpp"

#include <cstdint>

#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_timer.h"

namespace triwhirl::runtime {
namespace {

bool IRAM_ATTR motorReleaseAlarmCallback(
    gptimer_handle_t, const gptimer_alarm_event_data_t*, void* user_data) {
  const TaskHandle_t task = static_cast<TaskHandle_t>(user_data);
  if (task == nullptr) return false;
  BaseType_t high_priority_task_woken = pdFALSE;
  vTaskNotifyGiveFromISR(task, &high_priority_task_woken);
  return high_priority_task_woken == pdTRUE;
}

std::uint32_t nowUs32() {
  return static_cast<std::uint32_t>(esp_timer_get_time());
}

}  // namespace

bool MotorExecutionTask::valid() const {
  const MotorTaskTimingConfig timing_config{config_.service_period_us,
                                             config_.late_slack_us};
  return executor_ != nullptr && executor_->valid() &&
         observation_mailbox_ != nullptr && timing_config.valid() &&
         config_.task_name != nullptr && config_.task_name[0] != '\0' &&
         config_.stack_depth > 0U && config_.priority > 0U &&
         config_.priority < configMAX_PRIORITIES &&
         (config_.core_id == 0 || config_.core_id == 1) &&
         config_.release_interrupt_priority >= 0;
}

bool MotorExecutionTask::start() {
  if (!valid() || task_handle_ != nullptr) return false;
  const BaseType_t created = xTaskCreatePinnedToCore(
      &MotorExecutionTask::taskThunk, config_.task_name, config_.stack_depth,
      this, config_.priority, &task_handle_, config_.core_id);
  if (created != pdPASS) {
    task_handle_ = nullptr;
    publishLifecycle(false, true, false);
    return false;
  }
  return true;
}

void MotorExecutionTask::taskThunk(void* context) {
  auto* self = static_cast<MotorExecutionTask*>(context);
  if (self == nullptr) {
    vTaskDelete(nullptr);
    return;
  }
  self->taskMain();
}

bool MotorExecutionTask::initReleaseClock() {
  gptimer_config_t timer_config{};
  timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  timer_config.direction = GPTIMER_COUNT_UP;
  timer_config.resolution_hz = 1000000U;
  timer_config.intr_priority = config_.release_interrupt_priority;

  gptimer_handle_t timer = nullptr;
  esp_err_t result = gptimer_new_timer(&timer_config, &timer);
  if (result == ESP_OK) {
    gptimer_event_callbacks_t callbacks{};
    callbacks.on_alarm = motorReleaseAlarmCallback;
    result = gptimer_register_event_callbacks(timer, &callbacks, task_handle_);
  }
  if (result == ESP_OK) {
    gptimer_alarm_config_t alarm{};
    alarm.alarm_count = config_.service_period_us;
    alarm.reload_count = 0U;
    alarm.flags.auto_reload_on_alarm = true;
    result = gptimer_set_alarm_action(timer, &alarm);
  }
  if (result == ESP_OK) result = gptimer_enable(timer);
  if (result == ESP_OK) result = gptimer_start(timer);

  if (result != ESP_OK) {
    if (timer != nullptr) {
      (void)gptimer_stop(timer);
      (void)gptimer_disable(timer);
      (void)gptimer_del_timer(timer);
    }
    return false;
  }

  release_timer_ = timer;
  return true;
}

void MotorExecutionTask::publishLifecycle(const bool running,
                                          const bool initialization_failed,
                                          const bool release_clock_failed) {
  MotorTaskTimingSnapshot snapshot = timing_tracker_.snapshot();
  snapshot.running = running;
  snapshot.initialization_failed = initialization_failed;
  snapshot.release_clock_failed = release_clock_failed;
  timing_mailbox_.publish(snapshot);
}

void MotorExecutionTask::taskMain() {
  // begin() executes in the pinned motor task, so any future SimpleFOC/AS5600
  // initialization remains outside the attitude-control domain.
  if (!executor_->begin(nowUs32())) {
    publishLifecycle(false, true, false);
    task_handle_ = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  // The GPTimer is created from the motor task itself. ESP-IDF therefore binds
  // its interrupt allocation to the same core as the pinned motor task instead
  // of contaminating the Core-1 attitude domain by construction.
  if (!initReleaseClock()) {
    publishLifecycle(false, false, true);
    task_handle_ = nullptr;
    vTaskDelete(nullptr);
    return;
  }

  timing_tracker_.reset();
  publishLifecycle(true, false, false);
  (void)ulTaskNotifyTake(pdTRUE, 0);

  while (true) {
    // Hardware release ticks accumulate as task notifications. Taking with
    // pdTRUE collapses backlog into a count rather than replaying stale service
    // invocations, preserving latest-value realtime semantics.
    const std::uint32_t notifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    timing_tracker_.recordReleaseNotifications(notifications, true);
    if (notifications == 0U) continue;

    const std::uint32_t start_us32 = nowUs32();
    executor_->service(start_us32);
    const std::uint32_t end_us32 = nowUs32();

    std::uint32_t command_apply_latency_us = 0U;
    MotorObservationSnapshot observation{};
    if (observation_mailbox_->tryRead(&observation) &&
        observation.applied_command_generation != 0U) {
      command_apply_latency_us = observation.command_apply_latency_us;
    }

    timing_tracker_.recordIteration(start_us32, end_us32,
                                    command_apply_latency_us);
    publishLifecycle(true, false, false);
  }
}

}  // namespace triwhirl::runtime
