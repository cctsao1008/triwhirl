#pragma once

#include "triwhirl/balance_controller.hpp"

namespace triwhirl::runtime {

enum class BalanceStartFailure : int {
  kNone = 0,
  kAlreadyActive,
  kMotorActive,
  kReleaseClock,
  kControllerConfig,
  kMotorConfig,
  kEncoder,
  kImu,
  kAttitude,
  kSafetyFault,
  kOutsideCapture,
  kWheelRate,
};

struct RuntimeBalanceStatus {
  bool configured = false;
  bool active = false;
  triwhirl::BalanceControllerConfig config{};
  triwhirl::BalanceControllerOutput output{};
};

bool configureRuntimeBalance(const triwhirl::BalanceControllerConfig& config);
BalanceStartFailure startRuntimeBalance(float* initial_vq_v = nullptr);
void stopRuntimeBalance();
void updateRuntimeBalance();
bool runtimeBalanceActive();
RuntimeBalanceStatus runtimeBalanceStatus();
const char* balanceStartFailureName(BalanceStartFailure failure);

}  // namespace triwhirl::runtime
