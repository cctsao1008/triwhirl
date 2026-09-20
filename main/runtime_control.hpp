#pragma once

namespace triwhirl::runtime {

// Core-1 realtime control task entry. runtime_main.cpp owns the implementation
// while the final legacy app_main.cpp state bridge is extracted.
void realtimeControlTask(void* opaque);

}  // namespace triwhirl::runtime
