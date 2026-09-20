#pragma once

#include <cstdint>
#include <type_traits>

namespace triwhirl::runtime {

enum class RuntimeReplyCode : std::uint8_t {
  kNone = 0,
  kSwingOwnsRealtime,
  kMotorStopOk,
  kStopOk,
  kSwingAlreadyInactive,
  kSwingAbortOk,
  kTimingResetOk,
  kTimingProfileResetOk,
  kFaultAlreadyClear,
  kFaultClearRejected,
  kFaultClearOk,
  kTelemetryOnOk,
  kTelemetryOffOk,
  kInvalidMotorConfig,
  kMotorConfigOk,
  kAttitudeResetFromAccelOk,
  kInvalidAttitudeAngle,
  kAttitudeResetAngleOk,
  kInvalidImuMap,
  kImuMapOk,
};

struct RuntimeReply {
  RuntimeReplyCode code = RuntimeReplyCode::kNone;
  bool prompt_after = false;

  int value0 = 0;
  int value1 = 0;
  int value2 = 0;
  int value3 = 0;
  int value4 = 0;
  int value5 = 0;
  float float0 = 0.0F;
};

static_assert(std::is_trivially_copyable_v<RuntimeReply>,
              "RuntimeReply must remain trivially copyable");
static_assert(sizeof(RuntimeReply) <= 40U,
              "RuntimeReply grew beyond the bounded egress budget");

}  // namespace triwhirl::runtime
