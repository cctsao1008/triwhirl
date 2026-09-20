#pragma once

#include <cstddef>
#include <cstdint>

#include "runtime_command.hpp"

namespace triwhirl::runtime {

constexpr std::size_t kSupervisorCommandBytes = 128U;

enum class SupervisorInputEventType : std::uint8_t {
  kRuntimeCommand,
  kSupervisorHandled,
  kLineOverflow,
};

struct SupervisorInputEvent {
  SupervisorInputEventType type = SupervisorInputEventType::kSupervisorHandled;
  RuntimeCommand runtime_command{};
};

using SupervisorWriteFn = void (*)(void* context, const char* data,
                                   std::size_t length);

// Starts the non-realtime supervisor ingress task. UART0 is retained as a wired
// development/service CLI. BLE ingress is NimBLE GATT RX-characteristic data
// drained through the BLE component's internal stream buffer; it is not UART.
// Read-only supervisor-owned commands are answered directly; all remaining
// command grammar is parsed into fixed-size RuntimeCommand records before it
// crosses into realtime. Raw command strings never cross this boundary.
bool initSupervisorIo(SupervisorWriteFn write_fn, void* write_context,
                      int core_id, unsigned task_priority);

// Non-blocking receive for the realtime consumer. At most one queue element is
// removed per call.
bool tryReceiveSupervisorInput(SupervisorInputEvent* event);

}  // namespace triwhirl::runtime
