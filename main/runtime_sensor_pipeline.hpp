#pragma once

#include <cstdint>

#include "runtime_sensor_frame.hpp"

namespace triwhirl::runtime {

struct RuntimeSensorPipelineStats {
  std::uint64_t requests = 0U;
  std::uint64_t request_drops = 0U;
  std::uint64_t request_overwrites = 0U;
  std::uint64_t frames_published = 0U;
  std::uint64_t incomplete_frames = 0U;
  std::uint32_t last_published_sequence = 0U;
};

// Child AS5600/MPU6050 acquisition workers must already be initialized. This
// coordinator runs on the I/O core, dispatches both workers as one generation,
// joins them outside the realtime deadline, and overwrites one latest-frame
// mailbox for Core 1. The request side is also latest-only: if the coordinator
// is still busy, a newer Core-1 request replaces the queued older generation.
bool initSensorFramePipeline(bool imu_enabled, int core_id,
                             unsigned task_priority);
bool dispatchSensorFrameAcquisition(std::uint32_t* sequence);

// Core 1 is the sole runtime consumer of the latest-frame mailbox. Every
// successful read also records the exact frame consumed by that control
// iteration. Balance queries that consumed copy rather than peeking the mailbox
// a second time, so a concurrent Core-0 overwrite cannot create a TOCTOU gap.
bool readLatestSensorFrame(RuntimeSensorFrame* frame);
bool readLastConsumedSensorFrame(RuntimeSensorFrame* frame);

RuntimeSensorPipelineStats sensorFramePipelineStats();
void resetSensorFramePipelineStats();

}  // namespace triwhirl::runtime
