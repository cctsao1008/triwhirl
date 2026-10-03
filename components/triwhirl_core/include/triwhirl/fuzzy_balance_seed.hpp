#pragma once

#include <algorithm>
#include <array>
#include <cstddef>

#include "triwhirl/fuzzy.hpp"

namespace triwhirl::fuzzy_balance {

// Qualitative normalized rule seed for the first pure-fuzzy near-upright
// controller milestone.
//
// This is deliberately not a hardware tune and contains no plant-derived,
// LQR, PID, or H-infinity gains. Physical influence is still set by the
// independently configured input normalization scales and target-velocity
// limit in FuzzyBalanceConfig.
//
// Linguistic term rank:
//   NL=-2, NS=-1, ZE=0, PS=+1, PL=+2
//
// Rule semantics:
//   1. theta_error and theta_rate form the attitude-restoring/damping intent;
//   2. that intent saturates linguistically at PL/NL rather than growing
//      linearly without bound;
//   3. wheel momentum may trim the command only while attitude urgency is ZE
//      or one linguistic step away from ZE;
//   4. mirrored state produces mirrored command and ZE/ZE/ZE -> 0.
//
// Canonical sign convention: positive local attitude intent requests negative
// wheel target velocity. Hardware sensor/motor polarity must be established
// independently before this seed is enabled on a real unit.
inline int termRank(const std::size_t term) {
  return static_cast<int>(term) - 2;
}

inline int signOfRank(const int value) {
  return (value > 0) - (value < 0);
}

inline float qualitativeSingleton(const std::size_t theta_term,
                                  const std::size_t theta_rate_term,
                                  const std::size_t wheel_term) {
  const int theta_rank = termRank(theta_term);
  const int rate_rank = termRank(theta_rate_term);
  const int wheel_rank = termRank(wheel_term);

  // Equal linguistic priority here does not imply equal physical gain: each
  // input has its own normalization scale in FuzzyBalanceConfig.
  const int attitude_urgency =
      std::clamp(theta_rank + rate_rank, -2, 2);
  int command_rank = -attitude_urgency;

  // Near equilibrium, use wheel velocity as momentum-centering intent. During
  // strong attitude recovery, preserve the restoring command and do not let a
  // stored wheel-speed term reverse it.
  if (attitude_urgency >= -1 && attitude_urgency <= 1) {
    command_rank =
        std::clamp(command_rank - signOfRank(wheel_rank), -2, 2);
  }

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
