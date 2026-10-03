#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

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
// The wheel input and target output are normalized by independent physical
// scales. The wheel baseline must therefore include
//
//   wheel_velocity_scale_rad_s / target_velocity_limit_rad_s
//
// or an identity rule in linguistic space would NOT be an identity in physical
// rad/s when the two scales differ.
//
// Rule semantics:
//   1. wheel state supplies the scale-correct absolute target baseline;
//   2. theta_error and theta_rate form restoring/damping urgency;
//   3. restoring/damping urgency shifts the target relative to that baseline;
//   4. the final normalized command is bounded to [-1,+1];
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
                                  const std::size_t wheel_term,
                                  const float wheel_scale_to_target_limit) {
  if (!std::isfinite(wheel_scale_to_target_limit) ||
      !(wheel_scale_to_target_limit > 0.0F)) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  const int theta_rank = termRank(theta_term);
  const int rate_rank = termRank(theta_rate_term);

  const int attitude_urgency =
      std::clamp(theta_rank + rate_rank, -2, 2);

  // Convert the wheel linguistic center back into an absolute target baseline
  // in normalized output coordinates. This is a coordinate conversion, not a
  // controller gain: when attitude urgency is zero, physical target velocity
  // equals physical wheel velocity wherever the target limit is not saturated.
  const float wheel_baseline_normalized =
      fuzzy::kFiveTermCenters[wheel_term] * wheel_scale_to_target_limit;
  const float attitude_correction_normalized =
      0.5F * static_cast<float>(attitude_urgency);

  return std::clamp(wheel_baseline_normalized -
                        attitude_correction_normalized,
                    -1.0F, 1.0F);
}

inline std::array<float, fuzzy::kFiveTermRuleCount3D>
makeQualitativeRuleSeed(const float wheel_velocity_scale_rad_s,
                        const float target_velocity_limit_rad_s) {
  std::array<float, fuzzy::kFiveTermRuleCount3D> singletons{};

  const float wheel_scale_to_target_limit =
      wheel_velocity_scale_rad_s / target_velocity_limit_rad_s;

  for (std::size_t i = 0; i < fuzzy::kFiveTermCount; ++i) {
    for (std::size_t j = 0; j < fuzzy::kFiveTermCount; ++j) {
      for (std::size_t k = 0; k < fuzzy::kFiveTermCount; ++k) {
        const std::size_t index =
            (i * fuzzy::kFiveTermCount + j) * fuzzy::kFiveTermCount + k;
        singletons[index] = qualitativeSingleton(
            i, j, k, wheel_scale_to_target_limit);
      }
    }
  }
  return singletons;
}

}  // namespace triwhirl::fuzzy_balance
