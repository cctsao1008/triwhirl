#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace triwhirl::runtime {

// Wait for the next 1 kHz hardware release. The first call lazily creates the
// GPTimer on core 0 so the periodic interrupt cannot preempt the core-1 control
// task. If timer setup fails, preserve the established FreeRTOS tick fallback
// for bring-up/diagnostic compatibility.
void waitForRealtimeRelease(TickType_t* previous_wake,
                            TickType_t fallback_increment);

}  // namespace triwhirl::runtime
