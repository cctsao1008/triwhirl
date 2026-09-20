#pragma once

namespace triwhirl::runtime {

// Single Core-1 realtime control task used by the application entry point.
void realtimeControlTask(void* opaque);

}  // namespace triwhirl::runtime
