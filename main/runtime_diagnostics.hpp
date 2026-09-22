#pragma once

#include <cstdint>

namespace triwhirl::runtime {

struct RuntimeSnapshot;

struct EncoderDiagnosticStatus {
  bool status_ok = false;
  std::uint8_t raw = 0U;
  bool magnet_detected = false;
  bool magnet_too_weak = false;
  bool magnet_too_strong = false;
  std::uint8_t agc = 0U;
  std::uint16_t magnitude = 0U;
};

// Called only by the Core-1 snapshot publisher. This copies the runtime-owned
// state needed by supervisor diagnostics into the bounded RuntimeSnapshot.
void populateRuntimeDiagnosticSnapshot(RuntimeSnapshot* snapshot);

// Called from the Core-0 supervisor domain. The AS5600 bus is owned by Core 0,
// so a live diagnostic status read does not enter the realtime control task.
bool readEncoderDiagnosticStatus(EncoderDiagnosticStatus* status);

}  // namespace triwhirl::runtime
