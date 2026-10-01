#include "triwhirl/motor_mailbox.hpp"

// Compile-only target-toolchain proof for the cross-domain transport contract.
// The header itself contains the lock-free 32-bit atomic requirement.  This
// translation unit deliberately creates no runtime task, mailbox instance, or
// ownership change; it only ensures the native Xtensa build accepts the exact
// contract that host CI stress-tests.
namespace {

[[maybe_unused]] void motorMailboxCompileProbe() {
  triwhirl::MotorCommandMailbox command_mailbox;
  triwhirl::MotorObservationMailbox observation_mailbox;
  triwhirl::MotorCommandSnapshot command{};
  triwhirl::MotorObservationSnapshot observation{};

  (void)command_mailbox.tryRead(&command);
  (void)observation_mailbox.tryRead(&observation);
}

}  // namespace
