#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace triwhirl::fuzzy {

constexpr std::size_t kFiveTermCount = 5;
constexpr std::array<float, kFiveTermCount> kFiveTermCenters{
    -1.0F, -0.5F, 0.0F, 0.5F, 1.0F};

struct FiveTermMembership {
  std::array<float, kFiveTermCount> weight{};
  bool input_clamped = false;
};

// Symmetric Ruspini partition on the normalized universe [-1, +1].
// Linguistic terms are ordered NL, NS, ZE, PS, PL. End terms use shoulders.
inline FiveTermMembership fiveTermMembership(const float normalized_value) {
  FiveTermMembership result{};
  if (!std::isfinite(normalized_value)) {
    return result;
  }

  const float x = std::clamp(normalized_value, -1.0F, 1.0F);
  result.input_clamped = x != normalized_value;
  constexpr float spacing = 0.5F;

  for (std::size_t i = 0; i < kFiveTermCenters.size(); ++i) {
    if (i == 0 && x <= kFiveTermCenters[i]) {
      result.weight[i] = 1.0F;
      continue;
    }
    if (i + 1 == kFiveTermCenters.size() && x >= kFiveTermCenters[i]) {
      result.weight[i] = 1.0F;
      continue;
    }
    const float distance = std::fabs(x - kFiveTermCenters[i]);
    result.weight[i] = std::max(0.0F, 1.0F - distance / spacing);
  }
  return result;
}

struct SugenoResult {
  bool valid = false;
  float value = 0.0F;
  float weight_sum = 0.0F;
  float max_rule_weight = 0.0F;
};

inline SugenoResult evaluateSugeno1D(
    const FiveTermMembership& input,
    const std::array<float, kFiveTermCount>& singletons) {
  SugenoResult result{};
  float weighted_sum = 0.0F;
  for (std::size_t i = 0; i < kFiveTermCount; ++i) {
    if (!std::isfinite(singletons[i])) return result;
    const float firing = input.weight[i];
    weighted_sum += firing * singletons[i];
    result.weight_sum += firing;
    result.max_rule_weight = std::max(result.max_rule_weight, firing);
  }
  if (!(result.weight_sum > 1.0e-6F) ||
      !std::isfinite(result.weight_sum) || !std::isfinite(weighted_sum)) {
    return result;
  }
  result.value = weighted_sum / result.weight_sum;
  result.valid = std::isfinite(result.value);
  return result;
}

// Zero-order Takagi-Sugeno inference using product firing strength and weighted
// average defuzzification. The caller owns the singleton rule surface.
inline SugenoResult evaluateSugeno2D(
    const FiveTermMembership& lhs, const FiveTermMembership& rhs,
    const std::array<float, kFiveTermCount * kFiveTermCount>& singletons) {
  SugenoResult result{};
  float weighted_sum = 0.0F;

  for (std::size_t i = 0; i < kFiveTermCount; ++i) {
    for (std::size_t j = 0; j < kFiveTermCount; ++j) {
      const float singleton = singletons[i * kFiveTermCount + j];
      if (!std::isfinite(singleton)) return result;
      const float firing = lhs.weight[i] * rhs.weight[j];
      weighted_sum += firing * singleton;
      result.weight_sum += firing;
      result.max_rule_weight = std::max(result.max_rule_weight, firing);
    }
  }

  if (!(result.weight_sum > 1.0e-6F) ||
      !std::isfinite(result.weight_sum) || !std::isfinite(weighted_sum)) {
    return result;
  }

  result.value = weighted_sum / result.weight_sum;
  result.valid = std::isfinite(result.value);
  return result;
}

}  // namespace triwhirl::fuzzy
