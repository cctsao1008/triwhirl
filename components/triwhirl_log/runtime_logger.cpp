#include "triwhirl/runtime_logger.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "esp_err.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

namespace triwhirl {
namespace log {
namespace {

constexpr std::size_t kRamBufferBytes = 32U * 1024U;
// Keep each runtime flash program to one common NOR page. The WROOM-32 data
// sheet gives 0.8 ms typical / 5 ms max page-program time. Autonomous swing
// experiments never program flash while the motor is active; the SRAM payload
// is drained only after stop().
constexpr std::size_t kFlashBatchBytes = 256U;
constexpr std::size_t kFlashSectorBytes = 4096U;
constexpr std::uint32_t kHeaderFlagComplete = 1U << 0;
constexpr std::uint32_t kCrc32Initial = 0xffffffffU;
// A 50 s / 12-capture swing run must fit in the fixed 32 KiB SRAM queue while
// flash is forbidden. Pump/rearm context at 10 Hz plus 250 Hz probe windows is
// under 31 KiB at the configured 160 ms * 12 maximum probe occupancy.
constexpr std::uint32_t kPumpRecordDecimation = 100U;
constexpr std::uint32_t kProbeRecordDecimation = 4U;

std::size_t roundUp(const std::size_t value, const std::size_t alignment) {
  return (value + alignment - 1U) / alignment * alignment;
}

std::uint32_t crc32Update(std::uint32_t crc,
                          const std::uint8_t* data,
                          const std::size_t length) {
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= static_cast<std::uint32_t>(data[i]);
    for (unsigned bit = 0; bit < 8U; ++bit) {
      const std::uint32_t mask =
          static_cast<std::uint32_t>(-(static_cast<std::int32_t>(crc & 1U)));
      crc = (crc >> 1U) ^ (0xedb88320U & mask);
    }
  }
  return crc;
}

const esp_partition_t* partitionFrom(const void* value) {
  return static_cast<const esp_partition_t*>(value);
}

StreamBufferHandle_t streamFrom(void* value) {
  return static_cast<StreamBufferHandle_t>(value);
}

TaskHandle_t taskFrom(void* value) {
  return static_cast<TaskHandle_t>(value);
}

}  // namespace

const char* loggerStateName(const LoggerState state) {
  switch (state) {
    case LoggerState::kUnavailable:
      return "unavailable";
    case LoggerState::kIdle:
      return "idle";
    case LoggerState::kErasing:
      return "erasing";
    case LoggerState::kReady:
      return "ready";
    case LoggerState::kRecording:
      return "recording";
    case LoggerState::kStopping:
      return "stopping";
    case LoggerState::kComplete:
      return "complete";
    case LoggerState::kError:
      return "error";
  }
  return "unknown";
}

bool RuntimeLogger::init(const char* partition_label) {
  if (partition_ != nullptr && stream_ != nullptr && worker_task_ != nullptr) {
    return true;
  }
  if (partition_label == nullptr) {
    return false;
  }

  const esp_partition_t* partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, partition_label);
  if (partition == nullptr || partition->size <= kTwLogFlashPayloadOffset) {
    setState(LoggerState::kUnavailable);
    return false;
  }

  StreamBufferHandle_t stream = xStreamBufferCreate(
      kRamBufferBytes, kFlashBatchBytes);
  if (stream == nullptr) {
    setState(LoggerState::kUnavailable);
    return false;
  }

  TaskHandle_t worker = nullptr;
  partition_ = partition;
  stream_ = stream;
  setState(LoggerState::kIdle);
  // Sensor workers remain at near-maximum priority. Priority 3 lets the logger
  // make deterministic forward progress after an experiment without competing
  // with realtime acquisition while flash writes are disabled during motion.
  if (xTaskCreatePinnedToCore(workerEntry, "triwhirl_log", 4096, this, 3,
                              &worker, 0) != pdPASS) {
    vStreamBufferDelete(stream);
    stream_ = nullptr;
    partition_ = nullptr;
    setState(LoggerState::kUnavailable);
    return false;
  }
  worker_task_ = worker;
  return true;
}

bool RuntimeLogger::prepare(const std::uint32_t max_records) {
  const esp_partition_t* partition = partitionFrom(partition_);
  const StreamBufferHandle_t stream = streamFrom(stream_);
  if (partition == nullptr || stream == nullptr || max_records == 0U) {
    return false;
  }
  const LoggerState state = state_;
  if (state == LoggerState::kErasing || state == LoggerState::kRecording ||
      state == LoggerState::kStopping) {
    return false;
  }

  const std::uint64_t payload_bytes =
      static_cast<std::uint64_t>(max_records) * kTwLogRecordBytes;
  const std::uint64_t required_bytes =
      static_cast<std::uint64_t>(kTwLogFlashPayloadOffset) + payload_bytes;
  if (required_bytes > partition->size) {
    return false;
  }

  const std::size_t erase_bytes = roundUp(
      static_cast<std::size_t>(required_bytes), kFlashSectorBytes);
  if (erase_bytes > partition->size) {
    return false;
  }

  xStreamBufferReset(stream);
  max_records_ = max_records;
  accepted_records_ = 0U;
  records_written_ = 0U;
  dropped_records_ = 0U;
  payload_crc32_ = kCrc32Initial;
  pump_record_counter_ = 0U;
  probe_record_counter_ = 0U;
  prepared_bytes_ = static_cast<std::uint32_t>(erase_bytes);
  flash_writes_allowed_ = true;
  setState(LoggerState::kErasing);
  if (worker_task_ != nullptr) {
    xTaskNotifyGive(taskFrom(worker_task_));
  }
  return true;
}

bool RuntimeLogger::start() {
  if (state_ != LoggerState::kReady || stream_ == nullptr) {
    return false;
  }
  xStreamBufferReset(streamFrom(stream_));
  accepted_records_ = 0U;
  records_written_ = 0U;
  dropped_records_ = 0U;
  payload_crc32_ = kCrc32Initial;
  pump_record_counter_ = 0U;
  probe_record_counter_ = 0U;
  flash_writes_allowed_ = true;
  setState(LoggerState::kRecording);
  return true;
}

bool RuntimeLogger::stop() {
  if (state_ == LoggerState::kComplete) {
    return true;
  }
  if (state_ != LoggerState::kRecording && state_ != LoggerState::kReady) {
    return false;
  }
  // Motor/autonomous code stops before requesting finalization. Flash can now
  // drain the bounded SRAM payload without injecting cache-off stalls into the
  // active control/sensor path.
  flash_writes_allowed_ = true;
  setState(LoggerState::kStopping);
  if (worker_task_ != nullptr) {
    xTaskNotifyGive(taskFrom(worker_task_));
  }
  return true;
}

bool RuntimeLogger::record(const RuntimeLogRecord& record_value) {
  const StreamBufferHandle_t stream = streamFrom(stream_);
  if (state_ != LoggerState::kRecording || stream == nullptr) {
    return false;
  }

  const bool pump_active = (record_value.flags & kRecordPumpActive) != 0U;
  const bool probe_active = (record_value.flags & kRecordProbeActive) != 0U;
  if (pump_active || probe_active) {
    // Autonomous motor identification is a hard no-flash region. This is more
    // conservative than only protecting ProbeActive: even one NOR page program
    // can make a 1 kHz I2C sample look stale and falsely latch sensor loss.
    flash_writes_allowed_ = false;
  }

  if (probe_active) {
    pump_record_counter_ = 0U;
    const std::uint32_t probe_index = probe_record_counter_++;
    if ((probe_index % kProbeRecordDecimation) != 0U) {
      return true;
    }
  } else if (pump_active) {
    probe_record_counter_ = 0U;
    const std::uint32_t pump_index = pump_record_counter_++;
    if ((pump_index % kPumpRecordDecimation) != 0U) {
      return true;
    }
  } else {
    pump_record_counter_ = 0U;
    probe_record_counter_ = 0U;
  }

  if (accepted_records_ >= max_records_) {
    dropped_records_ = dropped_records_ + 1U;
    return false;
  }

  const std::size_t sent = xStreamBufferSend(
      stream, &record_value, sizeof(record_value), 0);
  if (sent != sizeof(record_value)) {
    dropped_records_ = dropped_records_ + 1U;
    return false;
  }
  accepted_records_ = accepted_records_ + 1U;

  if (flash_writes_allowed_ &&
      xStreamBufferBytesAvailable(stream) >= kFlashBatchBytes &&
      worker_task_ != nullptr) {
    xTaskNotifyGive(taskFrom(worker_task_));
  }
  return true;
}

void RuntimeLogger::setFlashWritesAllowed(const bool allowed) {
  flash_writes_allowed_ = allowed;
  if (allowed && worker_task_ != nullptr) {
    xTaskNotifyGive(taskFrom(worker_task_));
  }
}

LoggerStatus RuntimeLogger::status() const {
  LoggerStatus result{};
  result.state = state_;
  const esp_partition_t* partition = partitionFrom(partition_);
  if (partition != nullptr) {
    result.partition_bytes = partition->size;
  }
  result.prepared_bytes = prepared_bytes_;
  result.max_records = max_records_;
  const StreamBufferHandle_t stream = streamFrom(stream_);
  if (stream != nullptr) {
    result.buffered_bytes = static_cast<std::uint32_t>(
        xStreamBufferBytesAvailable(stream));
  }
  result.records_written = records_written_;
  result.dropped_records = dropped_records_;
  result.logical_bytes = logicalSize();
  result.flash_writes_allowed = flash_writes_allowed_;
  return result;
}

bool RuntimeLogger::complete() const {
  return state_ == LoggerState::kComplete;
}

std::uint32_t RuntimeLogger::logicalSize() const {
  if (state_ != LoggerState::kComplete) {
    return 0U;
  }
  return static_cast<std::uint32_t>(
      kTwLogHeaderBytes + records_written_ * kTwLogRecordBytes);
}

bool RuntimeLogger::readLogical(const std::uint32_t offset,
                                void* data,
                                const std::size_t length) const {
  const esp_partition_t* partition = partitionFrom(partition_);
  if (partition == nullptr || data == nullptr || state_ != LoggerState::kComplete) {
    return false;
  }
  const std::uint32_t logical_size = logicalSize();
  if (offset > logical_size || length > logical_size - offset) {
    return false;
  }
  if (length == 0U) {
    return true;
  }

  auto* output = static_cast<std::uint8_t*>(data);
  std::uint32_t logical_offset = offset;
  std::size_t remaining = length;

  if (logical_offset < kTwLogHeaderBytes) {
    const std::size_t header_bytes = std::min<std::size_t>(
        remaining, kTwLogHeaderBytes - logical_offset);
    if (esp_partition_read(partition, logical_offset, output,
                           header_bytes) != ESP_OK) {
      return false;
    }
    output += header_bytes;
    remaining -= header_bytes;
    logical_offset += static_cast<std::uint32_t>(header_bytes);
  }

  if (remaining > 0U) {
    const std::uint32_t payload_offset =
        logical_offset - kTwLogHeaderBytes;
    if (esp_partition_read(partition,
                           kTwLogFlashPayloadOffset + payload_offset,
                           output, remaining) != ESP_OK) {
      return false;
    }
  }
  return true;
}

void RuntimeLogger::workerEntry(void* context) {
  auto* logger = static_cast<RuntimeLogger*>(context);
  if (logger != nullptr) {
    logger->workerLoop();
  }
  vTaskDelete(nullptr);
}

void RuntimeLogger::workerLoop() {
  std::uint8_t scratch[kFlashBatchBytes];
  while (true) {
    const LoggerState state = state_;
    if (state == LoggerState::kErasing) {
      const esp_partition_t* partition = partitionFrom(partition_);
      if (partition == nullptr || prepared_bytes_ == 0U ||
          esp_partition_erase_range(partition, 0U, prepared_bytes_) != ESP_OK) {
        setState(LoggerState::kError);
      } else {
        setState(LoggerState::kReady);
      }
      continue;
    }

    if (state == LoggerState::kRecording || state == LoggerState::kStopping) {
      const StreamBufferHandle_t stream = streamFrom(stream_);
      if (stream == nullptr) {
        setState(LoggerState::kError);
        continue;
      }

      const std::size_t available = xStreamBufferBytesAvailable(stream);
      const bool stopping = state == LoggerState::kStopping;
      const bool should_write = stopping ||
          (flash_writes_allowed_ && available >= kFlashBatchBytes);
      if (should_write && available > 0U) {
        std::size_t request = std::min<std::size_t>(available, sizeof(scratch));
        request -= request % kTwLogRecordBytes;
        if (request == 0U) {
          setState(LoggerState::kError);
          continue;
        }
        const std::size_t received = xStreamBufferReceive(
            stream, scratch, request, 0);
        if (received == 0U || received % kTwLogRecordBytes != 0U ||
            !writePayload(scratch, received)) {
          setState(LoggerState::kError);
          continue;
        }
        continue;
      }

      if (stopping && available == 0U) {
        if (!finalize()) {
          setState(LoggerState::kError);
        } else {
          setState(LoggerState::kComplete);
        }
        continue;
      }
    }

    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
  }
}

void RuntimeLogger::setState(const LoggerState state) {
  state_ = state;
}

bool RuntimeLogger::writePayload(const std::uint8_t* data,
                                 const std::size_t length) {
  const esp_partition_t* partition = partitionFrom(partition_);
  if (partition == nullptr || data == nullptr || length == 0U ||
      length % kTwLogRecordBytes != 0U) {
    return false;
  }
  const std::uint32_t record_count =
      static_cast<std::uint32_t>(length / kTwLogRecordBytes);
  if (records_written_ > max_records_ ||
      record_count > max_records_ - records_written_) {
    return false;
  }

  const std::size_t flash_offset = kTwLogFlashPayloadOffset +
      static_cast<std::size_t>(records_written_) * kTwLogRecordBytes;
  if (flash_offset + length > prepared_bytes_) {
    return false;
  }
  if (esp_partition_write(partition, flash_offset, data, length) != ESP_OK) {
    return false;
  }
  payload_crc32_ = crc32Update(payload_crc32_, data, length);
  records_written_ = records_written_ + record_count;
  return true;
}

bool RuntimeLogger::finalize() {
  const esp_partition_t* partition = partitionFrom(partition_);
  if (partition == nullptr || records_written_ != accepted_records_) {
    return false;
  }

  TwLogHeader header{};
  header.record_count = records_written_;
  header.payload_bytes = records_written_ * kTwLogRecordBytes;
  header.dropped_records = dropped_records_;
  header.payload_crc32 = payload_crc32_ ^ 0xffffffffU;
  header.flags = kHeaderFlagComplete;
  header.reserved[0] = static_cast<std::uint32_t>(kTwLogFlashPayloadOffset);
  header.reserved[1] = prepared_bytes_;

  return esp_partition_write(partition, 0U, &header, sizeof(header)) == ESP_OK;
}

}  // namespace log
}  // namespace triwhirl
