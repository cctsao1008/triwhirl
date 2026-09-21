#pragma once

#include <cstdint>
#include <type_traits>

namespace triwhirl::runtime {

enum class RuntimeStateEventType : std::uint8_t {
  kNone = 0,
  kFaultLatched,
  kMotorCalibrationEncoderUnavailable,
  kMotorCalibrationNoMotion,
  kMotorCalibrationPolePairInvalid,
  kMotorCalibrationConfigInvalid,
  kMotorCalibrationComplete,
  kFocStoppedConfigUnavailable,
  kFocStoppedEncoderUnavailable,
  kImuCalibrationComplete,
};

static_assert(sizeof(int) == sizeof(std::int32_t),
              "Runtime state-event integer slots require a 32-bit int ABI");

struct RuntimeStateEvent {
  RuntimeStateEventType type = RuntimeStateEventType::kNone;
  int value0 = 0;
  int value1 = 0;
  std::uint32_t u32_0 = 0U;
  float float0 = 0.0F;
  float float1 = 0.0F;
  float float2 = 0.0F;
  float float3 = 0.0F;
};

static_assert(std::is_trivially_copyable_v<RuntimeStateEvent>,
              "RuntimeStateEvent must remain trivially copyable");
static_assert(sizeof(RuntimeStateEvent) <= 32U,
              "RuntimeStateEvent grew beyond the bounded egress budget");

}  // namespace triwhirl::runtime
