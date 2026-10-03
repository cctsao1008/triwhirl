#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>

#include "triwhirl/fuzzy.hpp"

namespace {

constexpr float kTol = 1.0e-5F;
constexpr float kContinuityTol = 5.0e-4F;

bool near(const float lhs, const float rhs, const float tol = kTol) {
  return std::fabs(lhs - rhs) <= tol;
}

int fail(const char* message) {
  std::cerr << "FAIL: " << message << '\n';
  return 1;
}

float membershipSum(const triwhirl::fuzzy::FiveTermMembership& membership) {
  float sum = 0.0F;
  for (const float weight : membership.weight) sum += weight;
  return sum;
}

bool allZero(const triwhirl::fuzzy::FiveTermMembership& membership) {
  for (const float weight : membership.weight) {
    if (weight != 0.0F) return false;
  }
  return true;
}

std::array<float, triwhirl::fuzzy::kFiveTermRuleCount2D>
makeSymmetric2DSurface() {
  std::array<float, triwhirl::fuzzy::kFiveTermRuleCount2D> surface{};
  for (std::size_t i = 0; i < triwhirl::fuzzy::kFiveTermCount; ++i) {
    for (std::size_t j = 0; j < triwhirl::fuzzy::kFiveTermCount; ++j) {
      surface[i * triwhirl::fuzzy::kFiveTermCount + j] =
          0.5F * (triwhirl::fuzzy::kFiveTermCenters[i] +
                  triwhirl::fuzzy::kFiveTermCenters[j]);
    }
  }
  return surface;
}

std::array<float, triwhirl::fuzzy::kFiveTermRuleCount3D>
makeSymmetric3DSurface() {
  std::array<float, triwhirl::fuzzy::kFiveTermRuleCount3D> surface{};
  for (std::size_t i = 0; i < triwhirl::fuzzy::kFiveTermCount; ++i) {
    for (std::size_t j = 0; j < triwhirl::fuzzy::kFiveTermCount; ++j) {
      for (std::size_t k = 0; k < triwhirl::fuzzy::kFiveTermCount; ++k) {
        const std::size_t index =
            (i * triwhirl::fuzzy::kFiveTermCount + j) *
                triwhirl::fuzzy::kFiveTermCount +
            k;
        surface[index] =
            (triwhirl::fuzzy::kFiveTermCenters[i] +
             triwhirl::fuzzy::kFiveTermCenters[j] +
             triwhirl::fuzzy::kFiveTermCenters[k]) /
            3.0F;
      }
    }
  }
  return surface;
}

}  // namespace

int main() {
  using triwhirl::fuzzy::evaluateSugeno1D;
  using triwhirl::fuzzy::evaluateSugeno2D;
  using triwhirl::fuzzy::evaluateSugeno3D;
  using triwhirl::fuzzy::fiveTermMembership;
  using triwhirl::fuzzy::kFiveTermCenters;

  // Ruspini partition, finite clamping and clamp observability.
  for (int sample = -1500; sample <= 1500; ++sample) {
    const float x = static_cast<float>(sample) / 1000.0F;
    const auto membership = fiveTermMembership(x);
    if (!near(membershipSum(membership), 1.0F)) {
      return fail("five-term membership must sum to one for finite input");
    }
    const bool should_clamp = x < -1.0F || x > 1.0F;
    if (membership.input_clamped != should_clamp) {
      return fail("finite out-of-range input must report clamping");
    }
  }

  // NL/NS/ZE/PS/PL mirror symmetry.
  for (int sample = 0; sample <= 1000; ++sample) {
    const float x = static_cast<float>(sample) / 1000.0F;
    const auto positive = fiveTermMembership(x);
    const auto negative = fiveTermMembership(-x);
    for (std::size_t i = 0; i < triwhirl::fuzzy::kFiveTermCount; ++i) {
      const std::size_t mirrored = triwhirl::fuzzy::kFiveTermCount - 1U - i;
      if (!near(positive.weight[i], negative.weight[mirrored])) {
        return fail("five-term membership mirror symmetry violated");
      }
    }
  }

  // Endpoint shoulders and out-of-range clamping.
  const auto low_endpoint = fiveTermMembership(-1.0F);
  const auto low_outside = fiveTermMembership(-2.0F);
  const auto high_endpoint = fiveTermMembership(1.0F);
  const auto high_outside = fiveTermMembership(2.0F);
  if (!near(low_endpoint.weight[0], 1.0F) ||
      !near(low_outside.weight[0], 1.0F) || !low_outside.input_clamped ||
      !near(high_endpoint.weight[4], 1.0F) ||
      !near(high_outside.weight[4], 1.0F) || !high_outside.input_clamped) {
    return fail("endpoint shoulder behavior violated");
  }

  // Non-finite inputs are rejected by producing no firing strength.
  const auto nan_membership =
      fiveTermMembership(std::numeric_limits<float>::quiet_NaN());
  const auto inf_membership =
      fiveTermMembership(std::numeric_limits<float>::infinity());
  if (!allZero(nan_membership) || !allZero(inf_membership)) {
    return fail("non-finite membership input must produce no firing strength");
  }

  // Membership functions are continuous at all internal linguistic centers.
  constexpr float epsilon = 1.0e-4F;
  for (std::size_t center = 1; center + 1 < kFiveTermCenters.size(); ++center) {
    const float x = kFiveTermCenters[center];
    const auto left = fiveTermMembership(x - epsilon);
    const auto right = fiveTermMembership(x + epsilon);
    for (std::size_t term = 0; term < triwhirl::fuzzy::kFiveTermCount; ++term) {
      if (std::fabs(left.weight[term] - right.weight[term]) >
          kContinuityTol) {
        return fail("membership discontinuity detected at linguistic center");
      }
    }
  }

  // 1-D zero-order Sugeno: identity surface, center behavior and symmetry.
  const std::array<float, triwhirl::fuzzy::kFiveTermCount> surface_1d =
      kFiveTermCenters;
  for (int sample = -1200; sample <= 1200; sample += 25) {
    const float x = static_cast<float>(sample) / 1000.0F;
    const float expected = std::fmax(-1.0F, std::fmin(1.0F, x));
    const auto result = evaluateSugeno1D(fiveTermMembership(x), surface_1d);
    if (!result.valid || !near(result.value, expected, 2.0e-5F) ||
        !near(result.weight_sum, 1.0F)) {
      return fail("1-D Sugeno identity/partition contract violated");
    }
    const auto mirrored =
        evaluateSugeno1D(fiveTermMembership(-x), surface_1d);
    if (!mirrored.valid || !near(result.value, -mirrored.value, 2.0e-5F)) {
      return fail("1-D Sugeno mirror symmetry violated");
    }
  }
  const auto center_1d =
      evaluateSugeno1D(fiveTermMembership(0.0F), surface_1d);
  if (!center_1d.valid || !near(center_1d.value, 0.0F)) {
    return fail("1-D Sugeno center must evaluate to zero");
  }

  // 2-D zero-order Sugeno: symmetric linear singleton surface.
  const auto surface_2d = makeSymmetric2DSurface();
  for (int first = -1000; first <= 1000; first += 200) {
    for (int second = -1000; second <= 1000; second += 250) {
      const float x = static_cast<float>(first) / 1000.0F;
      const float y = static_cast<float>(second) / 1000.0F;
      const auto result = evaluateSugeno2D(
          fiveTermMembership(x), fiveTermMembership(y), surface_2d);
      const auto mirrored = evaluateSugeno2D(
          fiveTermMembership(-x), fiveTermMembership(-y), surface_2d);
      if (!result.valid || !mirrored.valid ||
          !near(result.value, 0.5F * (x + y), 3.0e-5F) ||
          !near(result.value, -mirrored.value, 3.0e-5F)) {
        return fail("2-D Sugeno symmetry/center contract violated");
      }
    }
  }

  // 3-D zero-order Sugeno: symmetric linear singleton surface.
  const auto surface_3d = makeSymmetric3DSurface();
  for (int first = -1000; first <= 1000; first += 250) {
    for (int second = -1000; second <= 1000; second += 400) {
      for (int third = -1000; third <= 1000; third += 500) {
        const float x = static_cast<float>(first) / 1000.0F;
        const float y = static_cast<float>(second) / 1000.0F;
        const float z = static_cast<float>(third) / 1000.0F;
        const auto result = evaluateSugeno3D(
            fiveTermMembership(x), fiveTermMembership(y),
            fiveTermMembership(z), surface_3d);
        const auto mirrored = evaluateSugeno3D(
            fiveTermMembership(-x), fiveTermMembership(-y),
            fiveTermMembership(-z), surface_3d);
        if (!result.valid || !mirrored.valid ||
            !near(result.value, (x + y + z) / 3.0F, 4.0e-5F) ||
            !near(result.value, -mirrored.value, 4.0e-5F)) {
          return fail("3-D Sugeno symmetry/center contract violated");
        }
      }
    }
  }

  // Non-finite input membership must fail closed through inference.
  const auto invalid_input_result =
      evaluateSugeno1D(nan_membership, surface_1d);
  if (invalid_input_result.valid) {
    return fail("Sugeno inference must reject non-firing non-finite input");
  }

  // Every singleton is required to be finite, even if its rule does not fire.
  auto invalid_surface_1d = surface_1d;
  invalid_surface_1d[4] = std::numeric_limits<float>::quiet_NaN();
  if (evaluateSugeno1D(fiveTermMembership(0.0F), invalid_surface_1d).valid) {
    return fail("1-D Sugeno must reject non-finite singleton");
  }

  auto invalid_surface_2d = surface_2d;
  invalid_surface_2d[0] = std::numeric_limits<float>::infinity();
  if (evaluateSugeno2D(fiveTermMembership(0.0F), fiveTermMembership(0.0F),
                       invalid_surface_2d)
          .valid) {
    return fail("2-D Sugeno must reject non-finite singleton");
  }

  auto invalid_surface_3d = surface_3d;
  invalid_surface_3d[0] = std::numeric_limits<float>::quiet_NaN();
  if (evaluateSugeno3D(fiveTermMembership(0.0F), fiveTermMembership(0.0F),
                       fiveTermMembership(0.0F), invalid_surface_3d)
          .valid) {
    return fail("3-D Sugeno must reject non-finite singleton");
  }

  std::cout << "PASS: deterministic fuzzy core contract\n";
  return 0;
}
