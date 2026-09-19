#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace triwhirl {
namespace log {

constexpr std::uint32_t kTwLogMagic = 0x474c5754U;  // "TWLG" little-endian
constexpr std::uint16_t kTwLogVersion = 1U;
constexpr std::uint16_t kTwLogHeaderBytes = 64U;
constexpr std::uint16_t kTwLogRecordBytes = 32U;
constexpr std::uint16_t kTwLogSamplePeriodUs = 1000U;
constexpr std::size_t kTwLogFlashPayloadOffset = 4096U;

#pragma pack(push, 1)
struct TwLogHeader {
  std::uint32_t magic = kTwLogMagic;
  std::uint16_t version = kTwLogVersion;
  std::uint16_t header_size = kTwLogHeaderBytes;
  std::uint16_t record_size = kTwLogRecordBytes;
  std::uint16_t sample_period_us = kTwLogSamplePeriodUs;
  std::uint32_t record_count = 0U;
  std::uint32_t payload_bytes = 0U;
  std::uint32_t dropped_records = 0U;
  std::uint32_t payload_crc32 = 0U;
  std::uint32_t flags = 0U;
  std::uint32_t reserved[8]{};
};

struct RuntimeLogRecord {
  std::uint32_t t_us = 0U;
  float theta_rad = 0.0F;
  float theta_rate_rad_s = 0.0F;
  float wheel_rate_rad_s = 0.0F;
  float vq_v = 0.0F;
  float accel_weight = 0.0F;
  std::uint32_t fault_mask = 0U;
  std::uint16_t flags = 0U;
  std::uint16_t raw_count = 0U;
};
#pragma pack(pop)

static_assert(sizeof(TwLogHeader) == kTwLogHeaderBytes,
              "TWLG header must remain 64 bytes");
static_assert(sizeof(RuntimeLogRecord) == kTwLogRecordBytes,
              "TWLG record must remain 32 bytes");

enum RecordFlags : std::uint16_t {
  kRecordEncoderValid = 1U << 0,
  kRecordWheelRateValid = 1U << 1,
  kRecordImuValid = 1U << 2,
  kRecordAttitudeValid = 1U << 3,
  kRecordMotorActive = 1U << 4,
  kRecordMotorFoc = 1U << 5,
  kRecordMotorOpenLoop = 1U << 6,
  kRecordMotorCalibrating = 1U << 7,
  kRecordSafetyFaulted = 1U << 8,
  kRecordCriticalWindow = 1U << 9,
  kRecordVertexA = 1U << 10,
  kRecordVertexB = 1U << 11,
  kRecordVertexC = 1U << 12,
  kRecordProbeActive = 1U << 13,
  kRecordPumpActive = 1U << 14,
};

enum class LoggerState : std::uint8_t {
  kUnavailable,
  kIdle,
  kErasing,
  kReady,
  kRecording,
  kStopping,
  kComplete,
  kError,
};

struct LoggerStatus {
  LoggerState state = LoggerState::kUnavailable;
  std::uint32_t partition_bytes = 0U;
  std::uint32_t prepared_bytes = 0U;
  std::uint32_t max_records = 0U;
  std::uint32_t buffered_bytes = 0U;
  std::uint32_t records_written = 0U;
  std::uint32_t dropped_records = 0U;
  std::uint32_t logical_bytes = 0U;
  bool flash_writes_allowed = true;
};

class RuntimeLogger {
 public:
  bool init(const char* partition_label = "twlog");

  // Erase enough flash for a bounded recording. This is asynchronous because
  // sector erase can take tens to hundreds of milliseconds.
  bool prepare(std::uint32_t max_records);
  bool start();
  bool stop();

  // The control task calls record() once per control tick. This only copies the
  // fixed-size record into an SRAM stream buffer and never touches flash.
  bool record(const RuntimeLogRecord& record);

  // Autonomous control can block flash programming during a near-upright
  // critical window and re-enable it afterwards. SRAM absorbs the records while
  // programming is paused.
  void setFlashWritesAllowed(bool allowed);

  LoggerStatus status() const;
  bool complete() const;

  // Logical TWLG bytes are header followed immediately by payload records. The
  // physical flash layout reserves the first 4 KiB sector for the header.
  std::uint32_t logicalSize() const;
  bool readLogical(std::uint32_t offset, void* data, std::size_t length) const;

 private:
  static void workerEntry(void* context);
  void workerLoop();
  void setState(LoggerState state);
  bool writePayload(const std::uint8_t* data, std::size_t length);
  bool finalize();

  const void* partition_ = nullptr;
  void* stream_ = nullptr;
  void* worker_task_ = nullptr;

  volatile LoggerState state_ = LoggerState::kUnavailable;
  volatile bool flash_writes_allowed_ = true;
  volatile std::uint32_t prepared_bytes_ = 0U;
  volatile std::uint32_t max_records_ = 0U;
  volatile std::uint32_t accepted_records_ = 0U;
  volatile std::uint32_t records_written_ = 0U;
  volatile std::uint32_t dropped_records_ = 0U;
  volatile std::uint32_t payload_crc32_ = 0U;
};

const char* loggerStateName(LoggerState state);

}  // namespace log
}  // namespace triwhirl
