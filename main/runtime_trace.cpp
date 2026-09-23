#include "runtime_trace.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "triwhirl/ble_transport.hpp"
#include "triwhirl/upright_geometry.hpp"

#ifndef TRIWHIRL_GIT_SHA32
#define TRIWHIRL_GIT_SHA32 0U
#endif

#ifndef TRIWHIRL_GIT_DIRTY
#define TRIWHIRL_GIT_DIRTY 1
#endif

namespace triwhirl::runtime {
namespace {

constexpr std::uint32_t kTraceWriteTimeoutMs = 50U;
constexpr TickType_t kTraceIdleDelayTicks = pdMS_TO_TICKS(1);
constexpr float kRadToDeg = 180.0F / triwhirl::kPi;
constexpr float kTargetVelocityLimitRadS = 140.0F;
constexpr float kVqLimitV = 4.0F;
constexpr std::uint32_t kFirmwareGitSha32 = TRIWHIRL_GIT_SHA32;
constexpr bool kFirmwareGitDirty = TRIWHIRL_GIT_DIRTY != 0;

StandupTraceRecord trace_ring[kStandupTraceRingRecords]{};
std::atomic<std::uint32_t> trace_head{0U};
std::atomic<std::uint32_t> trace_tail{0U};
std::atomic<std::uint32_t> trace_sample_seq{0U};
std::atomic<std::uint32_t> trace_frame_seq{0U};
std::atomic<std::uint32_t> trace_dropped_records{0U};
std::atomic<std::uint32_t> trace_last_sample_us{0U};
std::atomic<std::uint32_t> trace_transport_drop_baseline{0U};
std::atomic<bool> trace_active{false};
std::atomic<bool> trace_start_pending{false};
std::atomic<bool> trace_end_pending{false};
std::atomic<bool> trace_initialized{false};

std::int16_t quantizeSigned(const float value, const float scale) {
  if (!std::isfinite(value)) return 0;
  const float scaled = value * scale;
  if (scaled >= 32767.0F) return 32767;
  if (scaled <= -32768.0F) return -32768;
  return static_cast<std::int16_t>(scaled >= 0.0F ? scaled + 0.5F
                                                  : scaled - 0.5F);
}

std::uint32_t crc32(const std::uint8_t* data, const std::size_t length) {
  std::uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (unsigned bit = 0U; bit < 8U; ++bit) {
      const std::uint32_t mask =
          static_cast<std::uint32_t>(-(static_cast<std::int32_t>(crc & 1U)));
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

bool sendFrame(const StandupTraceRecord* records,
               const std::uint8_t sample_count,
               const std::uint8_t frame_flags,
               const std::uint32_t first_sample_seq) {
  struct Frame {
    StandupTraceFrameHeader header{};
    StandupTraceRecord records[kStandupTraceRecordsPerFrame]{};
  } frame{};

  frame.header.flags = static_cast<std::uint8_t>(
      frame_flags | (kFirmwareGitDirty ? kTraceFrameFirmwareDirty : 0U));
  frame.header.frame_seq = trace_frame_seq.load(std::memory_order_relaxed);
  frame.header.first_sample_seq = first_sample_seq;
  frame.header.sample_count = sample_count;
  frame.header.dropped_records =
      trace_dropped_records.load(std::memory_order_relaxed);
  const std::uint32_t transport_now = triwhirl::ble::traceTxDroppedBytes();
  const std::uint32_t transport_baseline =
      trace_transport_drop_baseline.load(std::memory_order_relaxed);
  frame.header.transport_dropped_bytes = transport_now - transport_baseline;
  frame.header.firmware_git_sha32 = kFirmwareGitSha32;

  if (sample_count > 0U && records != nullptr) {
    std::memcpy(frame.records, records,
                static_cast<std::size_t>(sample_count) *
                    sizeof(StandupTraceRecord));
    frame.header.payload_crc32 = crc32(
        reinterpret_cast<const std::uint8_t*>(frame.records),
        static_cast<std::size_t>(sample_count) * sizeof(StandupTraceRecord));
  }

  const std::size_t bytes = sizeof(StandupTraceFrameHeader) +
                            static_cast<std::size_t>(sample_count) *
                                sizeof(StandupTraceRecord);
  const std::size_t sent = triwhirl::ble::traceWriteBlocking(
      reinterpret_cast<const std::uint8_t*>(&frame), bytes,
      kTraceWriteTimeoutMs);
  if (sent != bytes) return false;
  trace_frame_seq.fetch_add(1U, std::memory_order_relaxed);
  return true;
}

void traceWorker(void*) {
  StandupTraceRecord batch[kStandupTraceRecordsPerFrame]{};
  while (true) {
    bool progressed = false;

    if (trace_start_pending.load(std::memory_order_acquire)) {
      if (sendFrame(nullptr, 0U, kTraceFrameStart, 0U)) {
        trace_start_pending.store(false, std::memory_order_release);
        progressed = true;
      }
    }

    if (!trace_start_pending.load(std::memory_order_acquire)) {
      const std::uint32_t tail = trace_tail.load(std::memory_order_relaxed);
      const std::uint32_t head = trace_head.load(std::memory_order_acquire);
      const std::uint32_t available = head - tail;
      if (available > 0U) {
        const std::uint32_t count = std::min<std::uint32_t>(
            available, static_cast<std::uint32_t>(kStandupTraceRecordsPerFrame));
        for (std::uint32_t i = 0U; i < count; ++i) {
          batch[i] = trace_ring[(tail + i) % kStandupTraceRingRecords];
        }
        const std::uint8_t frame_flags =
            trace_dropped_records.load(std::memory_order_relaxed) > 0U
                ? kTraceFrameOverrun
                : 0U;
        if (sendFrame(batch, static_cast<std::uint8_t>(count), frame_flags,
                      batch[0].sample_seq)) {
          trace_tail.store(tail + count, std::memory_order_release);
          progressed = true;
        }
      }
    }

    const std::uint32_t drained_tail = trace_tail.load(std::memory_order_acquire);
    const std::uint32_t drained_head = trace_head.load(std::memory_order_acquire);
    if (!trace_active.load(std::memory_order_acquire) &&
        trace_end_pending.load(std::memory_order_acquire) &&
        !trace_start_pending.load(std::memory_order_acquire) &&
        drained_tail == drained_head) {
      const std::uint8_t frame_flags =
          static_cast<std::uint8_t>(kTraceFrameEnd |
              (trace_dropped_records.load(std::memory_order_relaxed) > 0U
                   ? kTraceFrameOverrun
                   : 0U));
      if (sendFrame(nullptr, 0U, frame_flags,
                    trace_sample_seq.load(std::memory_order_relaxed))) {
        trace_end_pending.store(false, std::memory_order_release);
        progressed = true;
      }
    }

    if (!progressed) {
      vTaskDelay(kTraceIdleDelayTicks);
    }
  }
}

}  // namespace

bool initRuntimeTrace() {
  bool expected = false;
  if (!trace_initialized.compare_exchange_strong(expected, true,
                                                  std::memory_order_acq_rel)) {
    return true;
  }
  if (xTaskCreatePinnedToCore(traceWorker, "triwhirl_trace", 4096, nullptr, 2,
                              nullptr, 0) != pdPASS) {
    trace_initialized.store(false, std::memory_order_release);
    return false;
  }
  return true;
}

void startRuntimeStandupTrace() {
  if (!trace_initialized.load(std::memory_order_acquire) && !initRuntimeTrace()) {
    trace_active.store(false, std::memory_order_release);
    return;
  }
  trace_active.store(false, std::memory_order_release);
  trace_head.store(0U, std::memory_order_relaxed);
  trace_tail.store(0U, std::memory_order_relaxed);
  trace_sample_seq.store(0U, std::memory_order_relaxed);
  trace_frame_seq.store(0U, std::memory_order_relaxed);
  trace_dropped_records.store(0U, std::memory_order_relaxed);
  trace_last_sample_us.store(0U, std::memory_order_relaxed);
  trace_transport_drop_baseline.store(triwhirl::ble::traceTxDroppedBytes(),
                                      std::memory_order_relaxed);
  trace_end_pending.store(false, std::memory_order_release);
  trace_start_pending.store(true, std::memory_order_release);
  trace_active.store(true, std::memory_order_release);
}

void recordRuntimeStandupTrace(const triwhirl::StandupControllerInput& input,
                               const triwhirl::StandupControllerOutput& output,
                               const bool safety_faulted) {
  if (!trace_active.load(std::memory_order_acquire)) return;

  StandupTraceRecord record{};
  record.sample_seq = trace_sample_seq.fetch_add(1U, std::memory_order_relaxed);
  const std::uint32_t previous_sample_us =
      trace_last_sample_us.exchange(input.now_us, std::memory_order_relaxed);
  const std::uint32_t dt_us = previous_sample_us == 0U
                                  ? 0U
                                  : input.now_us - previous_sample_us;
  const bool dt_clamped = dt_us > 0xFFFFU;
  record.dt_us = static_cast<std::uint16_t>(dt_clamped ? 0xFFFFU : dt_us);
  record.error_cdeg = quantizeSigned(output.theta_error_rad * kRadToDeg, 100.0F);
  record.theta_rate_mrad_s = quantizeSigned(input.theta_rate_rad_s, 1000.0F);
  record.filtered_rate_mrad_s =
      quantizeSigned(output.filtered_rate_rad_s, 1000.0F);
  record.wheel_rate_centi_rad_s =
      quantizeSigned(input.wheel_rate_rad_s, 100.0F);
  record.target_velocity_centi_rad_s =
      quantizeSigned(output.target_velocity_rad_s, 100.0F);
  record.velocity_error_centi_rad_s =
      quantizeSigned(output.velocity_error_rad_s, 100.0F);
  record.integral_mv = quantizeSigned(output.velocity_integral_v, 1000.0F);
  record.vq_target_mv = quantizeSigned(output.vq_target_v, 1000.0F);
  record.vq_applied_mv = quantizeSigned(output.vq_v, 1000.0F);
  record.flags = static_cast<std::uint16_t>(output.phase) & kTracePhaseMask;
  if (output.stable) record.flags |= kTraceStable;
  if (output.valid) record.flags |= kTraceValid;
  if (std::fabs(output.target_velocity_rad_s) >=
      kTargetVelocityLimitRadS - 0.01F) {
    record.flags |= kTraceTargetSaturated;
  }
  if (std::fabs(output.vq_target_v) >= kVqLimitV - 0.001F) {
    record.flags |= kTraceVqSaturated;
  }
  if (safety_faulted) record.flags |= kTraceSafetyFault;
  if (dt_clamped) record.flags |= kTraceDtClamped;

  const std::uint32_t head = trace_head.load(std::memory_order_relaxed);
  const std::uint32_t tail = trace_tail.load(std::memory_order_acquire);
  if ((head - tail) >= kStandupTraceRingRecords) {
    trace_dropped_records.fetch_add(1U, std::memory_order_relaxed);
    return;
  }
  trace_ring[head % kStandupTraceRingRecords] = record;
  trace_head.store(head + 1U, std::memory_order_release);
}

void stopRuntimeStandupTrace() {
  trace_active.store(false, std::memory_order_release);
  trace_end_pending.store(true, std::memory_order_release);
}

}  // namespace triwhirl::runtime
