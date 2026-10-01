#include "runtime_motor_task.hpp"
#include "triwhirl/motor_execution.hpp"
#include "triwhirl/motor_execution_timing.hpp"
#include "triwhirl/motor_mailbox.hpp"

// Compile-only target-toolchain proof for the cross-domain transport/execution
// contract and its FreeRTOS scheduling wrapper. This translation unit creates no
// live task and changes no runtime ownership; it only makes the native target
// compiler instantiate the same boundary types that host CI exercises.
namespace {

[[maybe_unused]] void motorMailboxCompileProbe() {
  triwhirl::MotorCommandMailbox command_mailbox;
  triwhirl::MotorObservationMailbox observation_mailbox;
  triwhirl::MotorCommandSnapshot command{};
  triwhirl::MotorObservationSnapshot observation{};
  triwhirl::MotorExecutionConfig execution_config{};
  triwhirl::MotorTaskTimingConfig timing_config{1000U, 100U};
  triwhirl::MotorTaskTimingTracker timing_tracker(timing_config);

  // Compile-only placeholders: this function is never called, and these values
  // intentionally carry no production scheduling authority.
  triwhirl::runtime::MotorExecutionTaskConfig task_config{};
  task_config.task_name = "triwhirl_motor_probe";
  task_config.stack_depth = 4096U;
  task_config.priority = configMAX_PRIORITIES - 2;
  task_config.core_id = 0;
  task_config.service_period_us = 1000U;
  task_config.late_slack_us = 100U;
  task_config.release_interrupt_priority = 1;

  (void)command_mailbox.tryRead(&command);
  (void)observation_mailbox.tryRead(&observation);
  (void)execution_config;
  (void)timing_tracker.valid();
  (void)task_config;
}

}  // namespace
