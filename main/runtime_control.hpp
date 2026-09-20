#pragma once

namespace triwhirl::runtime {

// Core-1 realtime control task entry. Startup code creates this task directly;
// runtime composition must not select it indirectly by task-name interception.
void realtimeControlTask(void* opaque);

}  // namespace triwhirl::runtime
