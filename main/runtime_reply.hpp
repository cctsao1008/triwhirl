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
  kSwingAlreadyActive,
  kSwingStartRequiresMotorStopped,
  kSwingStartRequiresReadyState,
  kSwingStartRequiresLogCapacity,
  kSwingStartRejected,
  kSwingStartOk,
  kSwingConfigUsageError,
  kSwingConfigOk,
  kSwingStatus,
  kSwingTransitionEvent,
  kTimingResetOk,
  kTimingProfileOnOk,
  kTimingProfileOffOk,
  kTimingProfileResetOk,
  kTimingProfileHeader,
  kTimingProfileStage,
  kTimingProfileEnd,
  kFaultAlreadyClear,
  kFaultClearRejected,
  kFaultClearOk,
  kTelemetryOnOk,
  kTelemetryOffOk,
  kSafetyFaultLatched,
  kEncoderUnavailable,
  kInvalidRuntimeNumeric,
  kInvalidMotorConfig,
  kMotorConfigOk,
  kMotorNotConfigured,
  kMotorFocOk,
  kMotorCalibrationEncoderUnavailable,
  kMotorCalibrationStarted,
  kFieldStopped,
  kFieldOk,
  kAttitudeResetFromAccelOk,
  kInvalidAttitudeAngle,
  kAttitudeResetAngleOk,
  kImuUnavailable,
  kImuCalibrationStarted,
  kInvalidImuMap,
  kImuMapOk,
  kLogPrepareRequiresMotorStopped,
  kLogPrepareSecondsInvalid,
  kLogPrepareDurationOutOfRange,
  kLogPrepareRejected,
  kLogPrepareOk,
  kLogStartRequiresReady,
  kLogStartOk,
  kLogCriticalOnOk,
  kLogCriticalOffOk,
  kLogStopRejected,
  kLogStopOk,
  kLogDumpAlreadyActive,
  kLogDumpRequiresComplete,
  kLogDumpRequiresMotorStopped,
  kLogDumpRequiresBleSubscription,
  kLogDumpTaskFailed,
};

enum class RuntimeTimingProfileStageId : std::uint8_t {
  kEncoder = 0,
  kImuAttitude,
  kSafetySwing,
  kMotor,
  kLog,
  kConsole,
  kTelemetry,
  kLoop,
  kEncoderI2cRaw,
  kEncoderI2cStatus,
  kMpuI2c,
  kMpuDecode,
};

struct RuntimeReply {
  RuntimeReplyCode code = RuntimeReplyCode::kNone;
  bool prompt_after = false;

  // Shared fixed-size payload slots. Individual reply codes define their
  // interpretation. Keeping one POD-like record avoids variable-size messages
  // or heap ownership at the realtime/supervisor boundary.
  std::int32_t value0 = 0;
  std::int32_t value1 = 0;
  std::int32_t value2 = 0;
  std::int32_t value3 = 0;
  std::int32_t value4 = 0;
  std::int32_t value5 = 0;
  std::int32_t value6 = 0;
  std::int32_t value7 = 0;
  std::int32_t value8 = 0;
  std::uint32_t u32_0 = 0U;
  std::uint32_t u32_1 = 0U;
  std::uint64_t wide0 = 0U;
  std::uint64_t wide1 = 0U;
  float float0 = 0.0F;
  float float1 = 0.0F;
  float float2 = 0.0F;
  float float3 = 0.0F;
  float float4 = 0.0F;
  float float5 = 0.0F;
  float float6 = 0.0F;
  float float7 = 0.0F;
  float float8 = 0.0F;
};

static_assert(std::is_trivially_copyable_v<RuntimeReply>,
              "RuntimeReply must remain trivially copyable");
static_assert(sizeof(RuntimeReply) <= 104U,
              "RuntimeReply grew beyond the bounded egress budget");

}  // namespace triwhirl::runtime
