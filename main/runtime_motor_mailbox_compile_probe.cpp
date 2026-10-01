#include "triwhirl/motor_execution.hpp"
#include "triwhirl/motor_mailbox.hpp"

// Compile-only target-toolchain proof for the cross-domain transport/execution
// contract. The headers contain the lock-free 32-bit atomic requirement. This
// translation unit deliberately creates no runtime task, live mailbox instance,
// or ownership change; it only ensures the native Xtensa build accepts the same
// types and execution-domain contract that host CI exercises.
namespace {

[[maybe_unused]] void motorMailboxCompileProbe() {
  triwhirl::MotorCommandMailbox command_mailbox;
  triwhirl::MotorObservationMailbox observation_mailbox;
  triwhirl::MotorCommandSnapshot command{};
  triwhirl::MotorObservationSnapshot observation{};
  triwhirl::MotorExecutionConfig execution_config{};

  (void)command_mailbox.tryRead(&command);
  (void)observation_mailbox.tryRead(&observation);
  (void)execution_config;
}

}  // namespace
