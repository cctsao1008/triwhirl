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
// IMU workers run independently; the coordinator blocks on their result queues
// instead of spinning, so IDLE0 remains schedulable while synchronous I2C is in
// flight.
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

RuntimeSensorFrame last_consumed_frame{};
bool last_consumed_frame_valid = false;

std::uint32_t remainingJoinBudgetUs(const std::int64_t deadline_us) {
  const std::int64_t now_us = esp_timer_get_time();
  if (now_us >= deadline_us) {
    return 0U;
  }
  return static_cast<std::uint32_t>(deadline_us - now_us);
}

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

    if (encoder_dispatched) {
      frame.encoder_received = tryCollectEncoderAcquisition(
          encoder_sequence, &frame.encoder);
    }
    if (pipeline_imu_enabled && imu_dispatched) {
      frame.imu_received =
          tryCollectImuAcquisition(imu_sequence, &frame.imu);
    }

    if (pipeline_imu_enabled && imu_dispatched && !frame.imu_received) {
      const std::uint32_t remaining_us =
          remainingJoinBudgetUs(join_deadline_us);
      if (remaining_us > 0U) {
        frame.imu_received = collectImuAcquisition(
            imu_sequence, remaining_us, &frame.imu);
      }
    }

    if (encoder_dispatched && !frame.encoder_received) {
      frame.encoder_received = tryCollectEncoderAcquisition(
          encoder_sequence, &frame.encoder);
      if (!frame.encoder_received) {
        const std::uint32_t remaining_us =
            remainingJoinBudgetUs(join_deadline_us);
        if (remaining_us > 0U) {
          frame.encoder_received = collectEncoderAcquisition(
              encoder_sequence, remaining_us, &frame.encoder);
        }
      }
    }

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
  last_consumed_frame = {};
  last_consumed_frame_valid = false;
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
  if (frame == nullptr || frame_queue == nullptr ||
      xQueuePeek(frame_queue, frame, 0) != pdTRUE) {
    return false;
  }
  last_consumed_frame = *frame;
  last_consumed_frame_valid = true;
  return true;
}

bool readLastConsumedSensorFrame(RuntimeSensorFrame* const frame) {
  if (frame == nullptr || !last_consumed_frame_valid) {
    return false;
  }
  *frame = last_consumed_frame;
  return true;
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
