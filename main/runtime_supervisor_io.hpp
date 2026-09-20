#pragma once

#include <cstddef>
#include <cstdint>

#include "runtime_command.hpp"
#include "runtime_reply.hpp"

namespace triwhirl::runtime {

constexpr std::size_t kSupervisorCommandBytes = 128U;

struct SupervisorInputEvent {
  RuntimeCommand runtime_command{};
};

using SupervisorWriteFn = void (*)(void* context, const char* data,
                                   std::size_t length);

// Starts the non-realtime supervisor task. UART0 is retained as a wired
// development/service CLI. BLE ingress is NimBLE GATT RX-characteristic data
// drained through the BLE component's internal stream buffer; it is not UART.
// Read-only commands are answered directly on Core 0; mutation grammar is
// parsed into fixed-size RuntimeCommand records before crossing into realtime.
bool initSupervisorIo(SupervisorWriteFn write_fn, void* write_context,
                      int core_id, unsigned task_priority);

// Non-blocking receive for the realtime consumer. At most one command is
// removed per call. No raw command strings cross this boundary.
bool tryReceiveSupervisorInput(SupervisorInputEvent* event);

// Non-blocking Core-1 -> Core-0 result path. Realtime publishes structured
// outcomes only; the supervisor formats wire text and owns post-command prompts.
bool publishRuntimeReply(const RuntimeReply& reply);
std::uint32_t runtimeReplyDroppedCount();

}  // namespace triwhirl::runtime
