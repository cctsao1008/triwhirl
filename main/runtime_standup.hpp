#pragma once

#include <cstdint>

#include "triwhirl/standup_controller.hpp"

namespace triwhirl::runtime {

enum class StandupStartFailure : std::uint8_t {
  kNone = 0,
  kAlreadyActive,
  kMotorActive,
  kReleaseClock,
  kMotorConfig,
  kEncoder,
  kImu,
  kAttitude,
  kSafetyFault,
};

struct RuntimeStandupStatus {
  bool active = false;
  triwhirl::StandupControllerConfig config{};
  triwhirl::StandupControllerOutput output{};
};

bool configureRuntimeStandup(float theta_reference_rad);
StandupStartFailure startRuntimeStandup();
void stopRuntimeStandup();
void updateRuntimeStandup(std::uint32_t now_us);
bool runtimeStandupActive();
RuntimeStandupStatus runtimeStandupStatus();
const char* standupStartFailureName(StandupStartFailure failure);

}  // namespace triwhirl::runtime
