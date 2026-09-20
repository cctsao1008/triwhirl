#include "runtime_snapshot.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "runtime_diagnostics.hpp"

namespace triwhirl::runtime {
namespace {

QueueHandle_t snapshot_queue = nullptr;

}  // namespace

bool initRuntimeSnapshotChannel() {
  if (snapshot_queue != nullptr) {
    return false;
  }
  snapshot_queue = xQueueCreate(1U, sizeof(RuntimeSnapshot));
  return snapshot_queue != nullptr;
}

void publishRuntimeSnapshot(const RuntimeSnapshot& snapshot) {
  if (snapshot_queue == nullptr) {
    return;
  }
  RuntimeSnapshot complete = snapshot;
  populateRuntimeDiagnosticSnapshot(&complete);
  xQueueOverwrite(snapshot_queue, &complete);
}

bool readLatestRuntimeSnapshot(RuntimeSnapshot* const snapshot) {
  return snapshot != nullptr && snapshot_queue != nullptr &&
         xQueuePeek(snapshot_queue, snapshot, 0) == pdTRUE;
}

}  // namespace triwhirl::runtime
