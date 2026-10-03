#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string>

#include "triwhirl/fuzzy.hpp"

namespace {

constexpr float kTol = 2.0e-5F;

bool near(const float lhs, const float rhs, const float tolerance = kTol) {
  const float scale = 1.0F + std::max(std::fabs(lhs), std::fabs(rhs));
  return std::fabs(lhs - rhs) <= tolerance * scale;
}

bool expect(const bool condition, const std::string& name, int& failures) {
  if (condition) {
    std::cout << "PASS " << name << '\n';
    return true;
  }
  std::cerr << "FAIL " << name << '\n';
  ++failures;
  return false;
}

float membershipSum(const triwhirl::fuzzy::FiveTermMembership& membership) {
  float sum = 0.0F;
  for (const float weight : membership.weight) sum += weight;
  return sum;
}

std::array<float, triwhirl::fuzzy::kFiveTermRuleCount2D>
makeOddSymmetric2DSingletons() {
  std::array<float, triwhirl::fuzzy::kFiveTermRuleCount2D> table{};
  for (std::size_t i = 0; i < triwhirl::fuzzy::kFiveTermCount; ++i) {
    for (std::size_t j = 0; j < triwhirl::fuzzy::kFiveTermCount; ++j) {
      table[i * triwhirl::fuzzy::kFiveTermCount + j] =
          0.5F * (triwhirl::fuzzy::kFiveTermCenters[i] +
                  triwhirl::fuzzy::kFiveTermCenters[j]);
    }
  }
  return table;
}

std::array<float, triwhirl::fuzzy::kFiveTermRuleCount3D>
makeOddSymmetric3DSingletons() {
  std::array<float, triwhirl::fuzzy::kFiveTermRuleCount3D> table{};
  for (std::size_t i = 0; i < triwhirl::fuzzy::kFiveTermCount; ++i) {
    for (std::size_t j = 0; j < triwhirl::fuzzy::kFiveTermCount; ++j) {
      for (std::size_t k = 0; k < triwhirl::fuzzy::kFiveTermCount; ++k) {
        const std::size_t index =
            (i * triwhirl::fuzzy::kFiveTermCount + j) *
                triwhirl::fuzzy::kFiveTermCount +
            k;
        table[index] =
            (triwhirl::fuzzy::kFiveTermCenters[i] +
             triwhirl::fuzzy::kFiveTermCenters[j] +
             triwhirl::fuzzy::kFiveTermCenters[k]) /
            3.0F;
      }
    }
  }
  return table;
}

}  // namespace

int main() {
  int failures = 0;

  {
    bool partition_ok = true;
    bool finite_ok = true;
    for (int step = -100; step <= 100; ++step) {
      const float x = static_cast<float>(step) / 100.0F;
      const auto membership = triwhirl::fuzzy::fiveTermMembership(x);
      partition_ok = partition_ok && near(membershipSum(membership), 1.0F);
      for (const float weight : membership.weight) {
        finite_ok = finite_ok && std::isfinite(weight) && weight >= 0.0F &&
                    weight <= 1.0F;
      }
    }
    expect(partition_ok, "five-term memberships form a Ruspini partition",
           failures);
    expect(finite_ok, "five-term memberships stay finite and bounded", failures);
  }

  {
    bool mirror_ok = true;
    for (int step = 0; step <= 100; ++step) {
      const float x = static_cast<float>(step) / 100.0F;
      const auto positive = triwhirl::fuzzy::fiveTermMembership(x);
      const auto negative = triwhirl::fuzzy::fiveTermMembership(-x);
      for (std::size_t i = 0; i < triwhirl::fuzzy::kFiveTermCount; ++i) {
        mirror_ok = mirror_ok &&
                    near(positive.weight[i],
                         negative.weight[triwhirl::fuzzy::kFiveTermCount - 1U - i]);
      }
    }
    expect(mirror_ok, "five-term memberships are mirror symmetric", failures);
  }

  {
    const auto low = triwhirl::fuzzy::fiveTermMembership(-1.0F);
    const auto high = triwhirl::fuzzy::fiveTermMembership(1.0F);
    const auto below = triwhirl::fuzzy::fiveTermMembership(-4.0F);
    const auto above = triwhirl::fuzzy::fiveTermMembership(4.0F);
    expect(near(low.weight[0], 1.0F) && near(high.weight[4], 1.0F),
           "endpoint linguistic terms use shoulders", failures);
    expect(below.input_clamped && above.input_clamped &&
               near(below.weight[0], 1.0F) && near(above.weight[4], 1.0F) &&
               near(membershipSum(below), 1.0F) &&
               near(membershipSum(above), 1.0F),
           "finite inputs outside the universe clamp to endpoint shoulders",
           failures);
  }

  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    const auto nan_membership = triwhirl::fuzzy::fiveTermMembership(nan);
    const auto inf_membership = triwhirl::fuzzy::fiveTermMembership(inf);
    expect(near(membershipSum(nan_membership), 0.0F) &&
               near(membershipSum(inf_membership), 0.0F),
           "non-finite fuzzy inputs are rejected", failures);
  }

  const std::array<float, triwhirl::fuzzy::kFiveTermCount> linear_1d{{
      -1.0F, -0.5F, 0.0F, 0.5F, 1.0F,
  }};
  const auto table_2d = makeOddSymmetric2DSingletons();
  const auto table_3d = makeOddSymmetric3DSingletons();

  {
    bool linear_ok = true;
    bool odd_ok = true;
    for (int step = -100; step <= 100; ++step) {
      const float x = static_cast<float>(step) / 100.0F;
      const auto result = triwhirl::fuzzy::evaluateSugeno1D(
          triwhirl::fuzzy::fiveTermMembership(x), linear_1d);
      const auto mirrored = triwhirl::fuzzy::evaluateSugeno1D(
          triwhirl::fuzzy::fiveTermMembership(-x), linear_1d);
      linear_ok = linear_ok && result.valid && near(result.value, x) &&
                  near(result.weight_sum, 1.0F);
      odd_ok = odd_ok && mirrored.valid && near(result.value, -mirrored.value);
    }
    expect(linear_ok, "1-D Sugeno interpolation preserves the centered axis",
           failures);
    expect(odd_ok, "1-D Sugeno interpolation preserves odd symmetry", failures);
  }

  {
    const std::array<std::array<float, 2>, 5> cases{{
        {{0.0F, 0.0F}}, {{0.25F, -0.75F}}, {{-0.4F, 0.8F}},
        {{1.0F, -1.0F}}, {{-0.9F, -0.1F}},
    }};
    bool symmetry_ok = true;
    for (const auto& input : cases) {
      const auto result = triwhirl::fuzzy::evaluateSugeno2D(
          triwhirl::fuzzy::fiveTermMembership(input[0]),
          triwhirl::fuzzy::fiveTermMembership(input[1]), table_2d);
      const auto mirrored = triwhirl::fuzzy::evaluateSugeno2D(
          triwhirl::fuzzy::fiveTermMembership(-input[0]),
          triwhirl::fuzzy::fiveTermMembership(-input[1]), table_2d);
      symmetry_ok = symmetry_ok && result.valid && mirrored.valid &&
                    near(result.value, -mirrored.value) &&
                    near(result.weight_sum, 1.0F) &&
                    near(mirrored.weight_sum, 1.0F);
    }
    const auto center = triwhirl::fuzzy::evaluateSugeno2D(
        triwhirl::fuzzy::fiveTermMembership(0.0F),
        triwhirl::fuzzy::fiveTermMembership(0.0F), table_2d);
    expect(symmetry_ok && center.valid && near(center.value, 0.0F),
           "2-D Sugeno surface preserves center and odd mirror symmetry",
           failures);
  }

  {
    const std::array<std::array<float, 3>, 5> cases{{
        {{0.0F, 0.0F, 0.0F}}, {{0.25F, -0.75F, 0.5F}},
        {{-0.4F, 0.8F, -0.2F}}, {{1.0F, -1.0F, 0.5F}},
        {{-0.9F, -0.1F, 0.7F}},
    }};
    bool symmetry_ok = true;
    for (const auto& input : cases) {
      const auto result = triwhirl::fuzzy::evaluateSugeno3D(
          triwhirl::fuzzy::fiveTermMembership(input[0]),
          triwhirl::fuzzy::fiveTermMembership(input[1]),
          triwhirl::fuzzy::fiveTermMembership(input[2]), table_3d);
      const auto mirrored = triwhirl::fuzzy::evaluateSugeno3D(
          triwhirl::fuzzy::fiveTermMembership(-input[0]),
          triwhirl::fuzzy::fiveTermMembership(-input[1]),
          triwhirl::fuzzy::fiveTermMembership(-input[2]), table_3d);
      symmetry_ok = symmetry_ok && result.valid && mirrored.valid &&
                    near(result.value, -mirrored.value) &&
                    near(result.weight_sum, 1.0F) &&
                    near(mirrored.weight_sum, 1.0F);
    }
    const auto center = triwhirl::fuzzy::evaluateSugeno3D(
        triwhirl::fuzzy::fiveTermMembership(0.0F),
        triwhirl::fuzzy::fiveTermMembership(0.0F),
        triwhirl::fuzzy::fiveTermMembership(0.0F), table_3d);
    expect(symmetry_ok && center.valid && near(center.value, 0.0F),
           "3-D Sugeno surface preserves center and odd mirror symmetry",
           failures);
  }

  {
    constexpr float epsilon = 1.0e-5F;
    const std::array<float, 3> boundaries{{-0.5F, 0.0F, 0.5F}};
    bool continuity_ok = true;
    for (const float boundary : boundaries) {
      const auto left = triwhirl::fuzzy::evaluateSugeno1D(
          triwhirl::fuzzy::fiveTermMembership(boundary - epsilon), linear_1d);
      const auto right = triwhirl::fuzzy::evaluateSugeno1D(
          triwhirl::fuzzy::fiveTermMembership(boundary + epsilon), linear_1d);
      continuity_ok = continuity_ok && left.valid && right.valid &&
                      std::fabs(left.value - right.value) <= 4.0F * epsilon;
    }
    expect(continuity_ok, "Sugeno output is continuous at membership boundaries",
           failures);
  }

  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    auto bad_1d = linear_1d;
    auto bad_2d = table_2d;
    auto bad_3d = table_3d;
    bad_1d[2] = nan;
    bad_2d[7] = nan;
    bad_3d[63] = nan;
    expect(!triwhirl::fuzzy::evaluateSugeno1D(
                triwhirl::fuzzy::fiveTermMembership(0.0F), bad_1d)
                .valid &&
               !triwhirl::fuzzy::evaluateSugeno2D(
                triwhirl::fuzzy::fiveTermMembership(0.0F),
                triwhirl::fuzzy::fiveTermMembership(0.0F), bad_2d)
                .valid &&
               !triwhirl::fuzzy::evaluateSugeno3D(
                triwhirl::fuzzy::fiveTermMembership(0.0F),
                triwhirl::fuzzy::fiveTermMembership(0.0F),
                triwhirl::fuzzy::fiveTermMembership(0.0F), bad_3d)
                .valid,
           "non-finite Sugeno singleton tables fail closed", failures);
  }

  if (failures == 0) {
    std::cout << "PASS generic fuzzy inference contract" << '\n';
    return 0;
  }
  std::cerr << "FAIL generic fuzzy inference contract failures=" << failures
            << '\n';
  return 1;
}
