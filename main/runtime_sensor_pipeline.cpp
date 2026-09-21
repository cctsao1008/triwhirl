#include "runtime_sensor_pipeline.hpp"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "runtime_encoder_acquisition.hpp"
#include "runtime_imu_acquisition.hpp"

namespace triwhirl::runtime {
namespace {

constexpr UBaseType_t kSensorFrameRequestDepth = 1U;
constexpr UBaseType_t kSensorFrameMailboxDepth = 1U;
// One shared Core-0 join deadline bounds the complete generation. Encoder and
// IMU workers run independently, but the coordinator never spends a full join
// budget on each sensor sequentially.
constexpr std::uint32_t kSensorWorkerJoinBudgetUs = 1500U;

struct SensorFrameRequest {
  std::uint32_t sequence = 0U;
  std::uint32_t requested_at_us = 0U;
};

QueueHandle_t request_queue = nullptr;
QueueHandle_t frame_queue = nullptr;
TaskHandle_t pipeline_task = nullptr;
bool pipeline_imu_enabled = false;
std::uint32_t request_sequence = 0U;
portMUX_TYPE stats_mux = portMUX_INITIALIZER_UNLOCKED;
RuntimeSensorPipelineStats stats{};

void noteFrameResult(const bool complete, const std::uint32_t sequence) {
  portENTER_CRITICAL(&stats_mux);
  if (complete) {
    ++stats.frames_published;
    stats.last_published_sequence = sequence;
  } else {
    ++stats.incomplete_frames;
  }
  portEXIT_CRITICAL(&stats_mux);
}

void sensorFramePipelineTask(void*) {
  SensorFrameRequest request{};
  while (true) {
    if (xQueueReceive(request_queue, &request, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    RuntimeSensorFrame frame{};
    frame.sequence = request.sequence;
    frame.requested_at_us = request.requested_at_us;
    frame.imu_expected = pipeline_imu_enabled;

    std::uint32_t encoder_sequence = 0U;
    const bool encoder_dispatched =
        dispatchEncoderAcquisition(&encoder_sequence);
    std::uint32_t imu_sequence = 0U;
    const bool imu_dispatched =
        !pipeline_imu_enabled || dispatchImuAcquisition(&imu_sequence);

    const std::int64_t join_deadline_us =
        esp_timer_get_time() +
        static_cast<std::int64_t>(kSensorWorkerJoinBudgetUs);
    while (esp_timer_get_time() < join_deadline_us) {
      if (encoder_dispatched && !frame.encoder_received) {
        frame.encoder_received = tryCollectEncoderAcquisition(
            encoder_sequence, &frame.encoder);
      }
      if (pipeline_imu_enabled && imu_dispatched && !frame.imu_received) {
        frame.imu_received =
            tryCollectImuAcquisition(imu_sequence, &frame.imu);
      }
      if ((!encoder_dispatched || frame.encoder_received) &&
          (!pipeline_imu_enabled || !imu_dispatched || frame.imu_received)) {
        break;
      }
      taskYIELD();
    }

    // One final zero-budget probe closes the deadline race and records the
    // worker join timeout exactly once when a dispatched result is still absent.
    if (encoder_dispatched && !frame.encoder_received) {
      frame.encoder_received = collectEncoderAcquisition(
          encoder_sequence, 0U, &frame.encoder);
    }
    if (pipeline_imu_enabled && imu_dispatched && !frame.imu_received) {
      frame.imu_received = collectImuAcquisition(
          imu_sequence, 0U, &frame.imu);
    }

    const bool encoder_valid = frame.encoder_received && frame.encoder.ok;
    const bool imu_valid = !pipeline_imu_enabled ||
                           (frame.imu_received && frame.imu.ok);
    frame.complete = encoder_valid && imu_valid;
    frame.published_at_us = static_cast<std::uint32_t>(esp_timer_get_time());

    // Publish every acquisition attempt as one coherent generation. Core 1 may
    // continue using an independently valid member in bring-up modes, while
    // complete=false is an explicit safety input for future Balance mode.
    xQueueOverwrite(frame_queue, &frame);
    noteFrameResult(frame.complete, frame.sequence);
  }
}

}  // namespace

bool initSensorFramePipeline(const bool imu_enabled, const int core_id,
                             const unsigned task_priority) {
  if (request_queue != nullptr || frame_queue != nullptr ||
      pipeline_task != nullptr) {
    return false;
  }

  request_queue =
      xQueueCreate(kSensorFrameRequestDepth, sizeof(SensorFrameRequest));
  frame_queue =
      xQueueCreate(kSensorFrameMailboxDepth, sizeof(RuntimeSensorFrame));
  if (request_queue == nullptr || frame_queue == nullptr) {
    return false;
  }

  pipeline_imu_enabled = imu_enabled;
  return xTaskCreatePinnedToCore(
             sensorFramePipelineTask, "triwhirl_sensor_frame", 4096, nullptr,
             static_cast<UBaseType_t>(task_priority), &pipeline_task, core_id) ==
         pdPASS;
}

bool dispatchSensorFrameAcquisition(std::uint32_t* const sequence) {
  if (sequence == nullptr || request_queue == nullptr || pipeline_task == nullptr) {
    portENTER_CRITICAL(&stats_mux);
    ++stats.request_drops;
    portEXIT_CRITICAL(&stats_mux);
    return false;
  }

  SensorFrameRequest request{};
  request.sequence = ++request_sequence;
  request.requested_at_us = static_cast<std::uint32_t>(esp_timer_get_time());

  // Keep only the newest not-yet-consumed generation. This prevents Core 0 from
  // spending its next cycle on an older queued request after a slow sensor frame.
  const bool replacing_queued_request = uxQueueMessagesWaiting(request_queue) > 0U;
  xQueueOverwrite(request_queue, &request);

  portENTER_CRITICAL(&stats_mux);
  ++stats.requests;
  if (replacing_queued_request) {
    ++stats.request_overwrites;
  }
  portEXIT_CRITICAL(&stats_mux);
  *sequence = request.sequence;
  return true;
}

bool readLatestSensorFrame(RuntimeSensorFrame* const frame) {
  return frame != nullptr && frame_queue != nullptr &&
         xQueuePeek(frame_queue, frame, 0) == pdTRUE;
}

RuntimeSensorPipelineStats sensorFramePipelineStats() {
  portENTER_CRITICAL(&stats_mux);
  const RuntimeSensorPipelineStats copy = stats;
  portEXIT_CRITICAL(&stats_mux);
  return copy;
}

void resetSensorFramePipelineStats() {
  portENTER_CRITICAL(&stats_mux);
  stats = {};
  portEXIT_CRITICAL(&stats_mux);
}

}  // namespace triwhirl::runtime
