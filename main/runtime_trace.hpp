#pragma once

#include <cstddef>
#include <cstdint>

#include "triwhirl/standup_controller.hpp"

namespace triwhirl::runtime {

inline constexpr std::uint32_t kStandupTraceMagic = 0x52545754U;  // "TWTR"
inline constexpr std::uint8_t kStandupTraceVersion = 2U;
inline constexpr std::size_t kStandupTraceRingRecords = 512U;
inline constexpr std::size_t kStandupTraceRecordsPerFrame = 8U;

#pragma pack(push, 1)
struct StandupTraceRecord {
  std::uint32_t sample_seq = 0U;
  std::uint16_t dt_us = 0U;
  std::int16_t error_cdeg = 0;
  std::int16_t theta_rate_mrad_s = 0;
  std::int16_t filtered_rate_mrad_s = 0;
  std::int16_t wheel_rate_centi_rad_s = 0;
  std::int16_t target_velocity_centi_rad_s = 0;
  std::int16_t velocity_error_centi_rad_s = 0;
  std::int16_t integral_mv = 0;
  std::int16_t vq_target_mv = 0;
  std::int16_t vq_applied_mv = 0;
  std::uint16_t flags = 0U;
};

struct StandupTraceFrameHeader {
  std::uint32_t magic = kStandupTraceMagic;
  std::uint8_t version = kStandupTraceVersion;
  std::uint8_t record_size = sizeof(StandupTraceRecord);
  std::uint8_t sample_count = 0U;
  std::uint8_t flags = 0U;
  std::uint32_t frame_seq = 0U;
  std::uint32_t first_sample_seq = 0U;
  std::uint32_t dropped_records = 0U;
  std::uint32_t transport_dropped_bytes = 0U;
  std::uint32_t firmware_git_sha32 = 0U;
  std::uint32_t payload_crc32 = 0U;
};
#pragma pack(pop)

static_assert(sizeof(StandupTraceRecord) == 26U,
              "standup trace record must remain 26 bytes");
static_assert(sizeof(StandupTraceFrameHeader) == 32U,
              "standup trace frame header must remain 32 bytes");
static_assert(sizeof(StandupTraceFrameHeader) +
                      kStandupTraceRecordsPerFrame * sizeof(StandupTraceRecord) ==
                  240U,
              "standup trace frame must exactly fit the preferred BLE payload budget");

enum StandupTraceRecordFlags : std::uint16_t {
  kTracePhaseMask = 0x0003U,
  kTraceStable = 1U << 2,
  kTraceValid = 1U << 3,
  kTraceTargetSaturated = 1U << 4,
  kTraceVqSaturated = 1U << 5,
  kTraceSafetyFault = 1U << 6,
  kTraceDtClamped = 1U << 7,
};

enum StandupTraceFrameFlags : std::uint8_t {
  kTraceFrameStart = 1U << 0,
  kTraceFrameEnd = 1U << 1,
  kTraceFrameOverrun = 1U << 2,
  kTraceFrameFirmwareDirty = 1U << 3,
};

bool initRuntimeTrace();
void startRuntimeStandupTrace();
void recordRuntimeStandupTrace(const triwhirl::StandupControllerInput& input,
                               const triwhirl::StandupControllerOutput& output,
                               bool safety_faulted);
void stopRuntimeStandupTrace();

}  // namespace triwhirl::runtime
