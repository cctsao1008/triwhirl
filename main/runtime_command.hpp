#pragma once

#include <cstdint>

namespace triwhirl::runtime {

// Commands that have crossed the supervisor/realtime ownership boundary.
// UART development CLI and BLE GATT ingress are parsed/validated in the
// supervisor domain; realtime receives only this fixed-size record.
enum class RuntimeCommandType : std::uint8_t {
  kNone = 0,
  kMotorStop,
  kStop,
  kSwingAbort,
  kTimingReset,
  kFaultClear,
  kTelemetryOn,
  kTelemetryOff,
};

struct RuntimeCommand {
  RuntimeCommandType type = RuntimeCommandType::kNone;
};

}  // namespace triwhirl::runtime
