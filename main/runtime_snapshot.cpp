#include "runtime_snapshot.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

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
  xQueueOverwrite(snapshot_queue, &snapshot);
}

bool readLatestRuntimeSnapshot(RuntimeSnapshot* const snapshot) {
  return snapshot != nullptr && snapshot_queue != nullptr &&
         xQueuePeek(snapshot_queue, snapshot, 0) == pdTRUE;
}

}  // namespace triwhirl::runtime
