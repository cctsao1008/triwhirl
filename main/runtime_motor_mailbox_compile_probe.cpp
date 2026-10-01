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
  triwhirl::runtime::MotorExecutionTaskConfig task_config{};

  (void)command_mailbox.tryRead(&command);
  (void)observation_mailbox.tryRead(&observation);
  (void)execution_config;
  (void)timing_tracker.valid();
  (void)task_config;
}

}  // namespace
