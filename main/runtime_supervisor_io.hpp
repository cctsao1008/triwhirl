#pragma once

#include <cstddef>
#include <cstdint>

#include "runtime_command.hpp"

namespace triwhirl::runtime {

constexpr std::size_t kSupervisorCommandBytes = 128U;

enum class SupervisorInputEventType : std::uint8_t {
  kCommand,
  kRuntimeCommand,
  kReadOnlyHandled,
  kLineOverflow,
};

struct SupervisorInputEvent {
  SupervisorInputEventType type = SupervisorInputEventType::kCommand;
  RuntimeCommand runtime_command{};
  char line[kSupervisorCommandBytes]{};
};

using SupervisorWriteFn = void (*)(void* context, const char* data,
                                   std::size_t length);

// Starts the non-realtime supervisor ingress task. UART0 is retained as a wired
// development/service CLI. BLE ingress is NimBLE GATT RX-characteristic data
// drained through the BLE component's internal stream buffer; it is not UART.
// Selected read-only commands are answered from the latest bounded runtime
// snapshot. Migrated mutating commands are parsed into RuntimeCommand records;
// not-yet-migrated commands remain temporarily available through the legacy
// string event until that path is deleted.
bool initSupervisorIo(SupervisorWriteFn write_fn, void* write_context,
                      int core_id, unsigned task_priority);

// Non-blocking receive for the realtime consumer. At most one queue element is
// removed per call.
bool tryReceiveSupervisorInput(SupervisorInputEvent* event);

}  // namespace triwhirl::runtime
