#pragma once

#include <cstdint>

namespace triwhirl::runtime {

// Commands that have already crossed the supervisor/realtime ownership boundary.
// The supervisor parses and validates text; realtime receives only this fixed-size
// record and executes the state mutation.
enum class RuntimeCommandType : std::uint8_t {
  kNone = 0,
  kMotorStop,
  kSwingAbort,
};

struct RuntimeCommand {
  RuntimeCommandType type = RuntimeCommandType::kNone;
};

}  // namespace triwhirl::runtime
