#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "triwhirl/fuzzy.hpp"

namespace {

bool near(const float a, const float b, const float tolerance = 1.0e-5F) {
  return std::fabs(a - b) <= tolerance;
}

void require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL " << message << '\n';
    std::exit(1);
  }
  std::cout << "PASS " << message << '\n';
}

float membershipSum(const triwhirl::fuzzy::FiveTermMembership& m) {
  float sum = 0.0F;
  for (const float weight : m.weight) sum += weight;
  return sum;
}

}  // namespace

int main() {
  using triwhirl::fuzzy::evaluateSugeno1D;
  using triwhirl::fuzzy::evaluateSugeno2D;
  using triwhirl::fuzzy::fiveTermMembership;

  const auto zero = fiveTermMembership(0.0F);
  require(near(zero.weight[2], 1.0F), "zero belongs fully to ZE");
  require(near(membershipSum(zero), 1.0F),
          "five-term partition sums to one at zero");

  bool partition_valid = true;
  bool symmetric = true;
  for (int step = -200; step <= 200; ++step) {
    const float x = static_cast<float>(step) / 200.0F;
    const auto a = fiveTermMembership(x);
    const auto b = fiveTermMembership(-x);
    partition_valid = partition_valid && near(membershipSum(a), 1.0F, 1.0e-4F);
    for (std::size_t i = 0; i < triwhirl::fuzzy::kFiveTermCount; ++i) {
      const std::size_t mirrored = triwhirl::fuzzy::kFiveTermCount - 1U - i;
      symmetric = symmetric && near(a.weight[i], b.weight[mirrored], 1.0e-5F);
    }
  }
  require(partition_valid, "five-term Ruspini partition holds across domain");
  require(symmetric, "membership partition is bilaterally symmetric");

  const auto low = fiveTermMembership(-10.0F);
  const auto high = fiveTermMembership(10.0F);
  require(low.input_clamped && high.input_clamped,
          "out-of-domain inputs use deterministic shoulder clamping");
  require(near(low.weight[0], 1.0F) && near(high.weight[4], 1.0F),
          "shoulder clamping selects end terms");

  constexpr std::array<float, triwhirl::fuzzy::kFiveTermCount> one_d{
      -1.0F, -0.5F, 0.0F, 0.5F, 1.0F};
  const auto one_pos = evaluateSugeno1D(fiveTermMembership(0.35F), one_d);
  const auto one_neg = evaluateSugeno1D(fiveTermMembership(-0.35F), one_d);
  require(one_pos.valid && one_neg.valid,
          "1-D Sugeno inference remains valid inside domain");
  require(near(one_pos.value, -one_neg.value),
          "1-D symmetric singleton surface is antisymmetric");

  constexpr std::array<float,
                       triwhirl::fuzzy::kFiveTermCount *
                           triwhirl::fuzzy::kFiveTermCount>
      two_d{
          -1.0F, -0.75F, -0.50F, -0.25F, 0.0F,
          -0.75F, -0.50F, -0.25F, 0.0F, 0.25F,
          -0.50F, -0.25F, 0.0F, 0.25F, 0.50F,
          -0.25F, 0.0F, 0.25F, 0.50F, 0.75F,
          0.0F, 0.25F, 0.50F, 0.75F, 1.0F,
      };
  const auto two_pos = evaluateSugeno2D(
      fiveTermMembership(0.4F), fiveTermMembership(-0.2F), two_d);
  const auto two_neg = evaluateSugeno2D(
      fiveTermMembership(-0.4F), fiveTermMembership(0.2F), two_d);
  require(two_pos.valid && two_neg.valid,
          "2-D Sugeno inference remains valid inside domain");
  require(near(two_pos.value, -two_neg.value),
          "2-D symmetric singleton surface is antisymmetric");

  bool continuity_valid = true;
  float previous_value = evaluateSugeno1D(fiveTermMembership(-1.0F), one_d).value;
  float max_jump = 0.0F;
  for (int step = 1; step <= 2000; ++step) {
    const float x = -1.0F + 2.0F * static_cast<float>(step) / 2000.0F;
    const auto current = evaluateSugeno1D(fiveTermMembership(x), one_d);
    continuity_valid = continuity_valid && current.valid;
    max_jump = std::max(max_jump, std::fabs(current.value - previous_value));
    previous_value = current.value;
  }
  require(continuity_valid && max_jump < 0.002F,
          "Sugeno surface is continuous across membership boundaries");

  auto invalid_singletons = one_d;
  invalid_singletons[2] = std::numeric_limits<float>::quiet_NaN();
  const auto invalid = evaluateSugeno1D(zero, invalid_singletons);
  require(!invalid.valid, "non-finite singleton is rejected");

  std::cout << "PASS generic fuzzy inference foundation\n";
  return 0;
}
