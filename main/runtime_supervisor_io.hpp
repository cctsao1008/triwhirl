#pragma once

#include <cstddef>
#include <cstdint>

namespace triwhirl::runtime {

constexpr std::size_t kSupervisorCommandBytes = 128U;

enum class SupervisorInputEventType : std::uint8_t {
  kCommand,
  kLineOverflow,
};

struct SupervisorInputEvent {
  SupervisorInputEventType type = SupervisorInputEventType::kCommand;
  char line[kSupervisorCommandBytes]{};
};

using SupervisorWriteFn = void (*)(void* context, const char* data,
                                   std::size_t length);

// Starts the non-realtime UART/BLE input task. The task owns byte polling and
// line assembly; complete fixed-size events cross into the realtime domain via
// a bounded queue.
bool initSupervisorIo(SupervisorWriteFn write_fn, void* write_context,
                      int core_id, unsigned task_priority);

// Non-blocking receive for the realtime consumer. At most one queue element is
// removed per call.
bool tryReceiveSupervisorInput(SupervisorInputEvent* event);

}  // namespace triwhirl::runtime
