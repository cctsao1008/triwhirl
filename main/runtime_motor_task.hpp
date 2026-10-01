#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "triwhirl/motor_execution.hpp"
#include "triwhirl/motor_execution_timing.hpp"

namespace triwhirl::runtime {

struct MotorExecutionTaskConfig {
  const char* task_name = "triwhirl_motor";
  std::uint32_t stack_depth = 4096U;
  UBaseType_t priority = configMAX_PRIORITIES - 2;
  BaseType_t core_id = 0;
  std::uint32_t service_period_us = 1000U;
  std::uint32_t late_slack_us = 100U;
  int release_interrupt_priority = 1;
};

// Target-side FreeRTOS/GPTimer wrapper around the library-independent
// MotorExecutionDomain. This class defines scheduling and timing observability,
// but production code must explicitly instantiate/start it; merely linking this
// translation unit does not create a second motor owner.
class MotorExecutionTask {
 public:
  MotorExecutionTask(MotorExecutionDomain* executor,
                     MotorObservationMailbox* observation_mailbox,
                     const MotorExecutionTaskConfig& config)
      : executor_(executor),
        observation_mailbox_(observation_mailbox),
        config_(config),
        timing_tracker_({config.service_period_us, config.late_slack_us}) {}

  MotorExecutionTask(const MotorExecutionTask&) = delete;
  MotorExecutionTask& operator=(const MotorExecutionTask&) = delete;

  bool valid() const;
  bool start();
  bool readTiming(MotorTaskTimingSnapshot* out) const {
    return timing_mailbox_.tryRead(out);
  }

 private:
  static void taskThunk(void* context);
  void taskMain();
  bool initReleaseClock();
  void publishLifecycle(bool running, bool initialization_failed,
                        bool release_clock_failed);

  MotorExecutionDomain* executor_ = nullptr;
  MotorObservationMailbox* observation_mailbox_ = nullptr;
  MotorExecutionTaskConfig config_{};
  MotorTaskTimingTracker timing_tracker_;
  MotorTaskTimingMailbox timing_mailbox_{};
  TaskHandle_t task_handle_ = nullptr;
  void* release_timer_ = nullptr;
};

}  // namespace triwhirl::runtime
