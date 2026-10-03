#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

#include "triwhirl/fuzzy.hpp"

namespace triwhirl::fuzzy_balance {

// Qualitative normalized rule seed for the pure-fuzzy near-upright controller.
//
// This is deliberately not a hardware tune and contains no plant-derived,
// LQR, PID, or H-infinity gains. Physical influence is still set by the
// independently configured input normalization scales and target-velocity
// limit in FuzzyBalanceConfig.
//
// Linguistic term rank:
//   NL=-2, NS=-1, ZE=0, PS=+1, PL=+2
//
// Important command-coordinate semantic:
//   the controller emits an ABSOLUTE wheel target velocity, not wheel torque,
//   Vq, or wheel acceleration. Therefore, with theta_error=theta_rate=ZE, the
//   qualitative target should preserve the current wheel-velocity operating
//   point rather than command the opposite wheel direction.
//
// Rule semantics:
//   1. wheel rank supplies the absolute target-velocity baseline;
//   2. theta_error and theta_rate form restoring/damping urgency;
//   3. restoring/damping urgency shifts the target relative to that baseline;
//   4. the final linguistic command saturates at PL/NL;
//   5. mirrored state produces mirrored command and ZE/ZE/ZE -> 0.
//
// Momentum unloading is intentionally not encoded here as "command opposite
// wheel speed at zero attitude error". That would conflate an absolute
// velocity target with torque intent. Saturation-aware momentum management is
// a later full-fuzzy behavior.
//
// Canonical sign convention: positive local attitude urgency shifts the wheel
// target in the negative direction. Hardware sensor/motor polarity must be
// established independently before this seed is enabled on a real unit.
inline int termRank(const std::size_t term) {
  return static_cast<int>(term) - 2;
}

inline float qualitativeSingleton(const std::size_t theta_term,
                                  const std::size_t theta_rate_term,
                                  const std::size_t wheel_term) {
  const int theta_rank = termRank(theta_term);
  const int rate_rank = termRank(theta_rate_term);
  const int wheel_rank = termRank(wheel_term);

  // Equal linguistic priority here does not imply equal physical influence:
  // each input has its own normalization scale in FuzzyBalanceConfig.
  const int attitude_urgency =
      std::clamp(theta_rank + rate_rank, -2, 2);

  // Absolute target-velocity semantics: preserve the wheel operating point at
  // zero attitude urgency, then shift that target to create correcting wheel
  // acceleration through the downstream velocity servo.
  const int command_rank =
      std::clamp(wheel_rank - attitude_urgency, -2, 2);

  return 0.5F * static_cast<float>(command_rank);
}

inline std::array<float, fuzzy::kFiveTermRuleCount3D>
makeQualitativeRuleSeed() {
  std::array<float, fuzzy::kFiveTermRuleCount3D> singletons{};
  for (std::size_t i = 0; i < fuzzy::kFiveTermCount; ++i) {
    for (std::size_t j = 0; j < fuzzy::kFiveTermCount; ++j) {
      for (std::size_t k = 0; k < fuzzy::kFiveTermCount; ++k) {
        const std::size_t index =
            (i * fuzzy::kFiveTermCount + j) * fuzzy::kFiveTermCount + k;
        singletons[index] = qualitativeSingleton(i, j, k);
      }
    }
  }
  return singletons;
}

}  // namespace triwhirl::fuzzy_balance
