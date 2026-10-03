#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <string>

#include "triwhirl/fuzzy_balance_controller.hpp"

namespace {

bool near(const float lhs, const float rhs, const float tolerance = 1.0e-5F) {
  return std::fabs(lhs - rhs) <= tolerance;
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

triwhirl::FuzzyBalanceConfig symmetricFixture() {
  triwhirl::FuzzyBalanceConfig config{};
  config.theta_error_scale_rad = 0.2F;
  config.theta_rate_scale_rad_s = 2.0F;
  config.wheel_velocity_scale_rad_s = 20.0F;
  config.target_velocity_limit_rad_s = 12.0F;

  // Contract fixture only: an antisymmetric affine surface in normalized
  // coordinates. This is not a closed-loop tune or hardware-ready controller.
  for (std::size_t i = 0; i < triwhirl::fuzzy::kFiveTermCount; ++i) {
    for (std::size_t j = 0; j < triwhirl::fuzzy::kFiveTermCount; ++j) {
      for (std::size_t k = 0; k < triwhirl::fuzzy::kFiveTermCount; ++k) {
        const std::size_t index =
            (i * triwhirl::fuzzy::kFiveTermCount + j) *
                triwhirl::fuzzy::kFiveTermCount +
            k;
        config.target_velocity_singletons[index] =
            -(triwhirl::fuzzy::kFiveTermCenters[i] +
              triwhirl::fuzzy::kFiveTermCenters[j] +
              triwhirl::fuzzy::kFiveTermCenters[k]) /
            3.0F;
      }
    }
  }
  config.rule_surface_configured = true;
  return config;
}

}  // namespace

int main() {
  int failures = 0;

  {
    const triwhirl::FuzzyBalanceController controller(
        triwhirl::FuzzyBalanceConfig{});
    expect(!controller.valid() &&
               !controller.evaluate(triwhirl::FuzzyBalanceInput{}).valid,
           "default configuration fails closed", failures);
  }

  const triwhirl::FuzzyBalanceConfig config = symmetricFixture();
  const triwhirl::FuzzyBalanceController controller(config);
  expect(controller.valid(), "explicit finite fixture is valid", failures);

  {
    const auto output = controller.evaluate(triwhirl::FuzzyBalanceInput{});
    expect(output.valid && near(output.target_velocity_normalized, 0.0F) &&
               near(output.target_velocity_rad_s, 0.0F) &&
               near(output.rule_weight_sum, 1.0F) &&
               near(output.max_rule_weight, 1.0F),
           "zero state maps to zero target at the ZE center", failures);
  }

  {
    const triwhirl::FuzzyBalanceInput positive{0.10F, 0.50F, 5.0F};
    const triwhirl::FuzzyBalanceInput negative{-0.10F, -0.50F, -5.0F};
    const auto positive_output = controller.evaluate(positive);
    const auto negative_output = controller.evaluate(negative);
    expect(positive_output.valid && negative_output.valid &&
               near(positive_output.target_velocity_normalized, -1.0F / 3.0F) &&
               near(positive_output.target_velocity_rad_s, -4.0F) &&
               near(negative_output.target_velocity_normalized,
                    -positive_output.target_velocity_normalized) &&
               near(negative_output.target_velocity_rad_s,
                    -positive_output.target_velocity_rad_s),
           "mirrored mechanical state produces mirrored target velocity",
           failures);
  }

  {
    const auto output = controller.evaluate(
        triwhirl::FuzzyBalanceInput{2.0F, 20.0F, 200.0F});
    expect(output.valid && output.input_clamped[0] && output.input_clamped[1] &&
               output.input_clamped[2] &&
               near(output.requested_normalized_input[0], 10.0F) &&
               near(output.requested_normalized_input[1], 10.0F) &&
               near(output.requested_normalized_input[2], 10.0F) &&
               near(output.applied_normalized_input[0], 1.0F) &&
               near(output.applied_normalized_input[1], 1.0F) &&
               near(output.applied_normalized_input[2], 1.0F) &&
               near(output.target_velocity_normalized, -1.0F) &&
               near(output.target_velocity_rad_s,
                    -config.target_velocity_limit_rad_s),
           "normalization clamps are observable and physical target is bounded",
           failures);
  }

  {
    const auto nan = std::numeric_limits<float>::quiet_NaN();
    const auto inf = std::numeric_limits<float>::infinity();
    expect(!controller.evaluate(triwhirl::FuzzyBalanceInput{nan, 0.0F, 0.0F})
                .valid &&
               !controller.evaluate(
                    triwhirl::FuzzyBalanceInput{0.0F, inf, 0.0F})
                    .valid &&
               !controller.evaluate(
                    triwhirl::FuzzyBalanceInput{0.0F, 0.0F, -inf})
                    .valid,
           "non-finite mechanical state fails closed", failures);
  }

  {
    auto invalid = config;
    invalid.rule_surface_configured = false;
    expect(!triwhirl::validFuzzyBalanceConfig(invalid),
           "rule surface requires explicit configuration", failures);

    invalid = config;
    invalid.theta_error_scale_rad = 0.0F;
    expect(!triwhirl::validFuzzyBalanceConfig(invalid),
           "non-positive normalization scale is rejected", failures);

    invalid = config;
    invalid.target_velocity_limit_rad_s =
        std::numeric_limits<float>::infinity();
    expect(!triwhirl::validFuzzyBalanceConfig(invalid),
           "non-finite physical target limit is rejected", failures);

    invalid = config;
    invalid.target_velocity_singletons[0] =
        std::numeric_limits<float>::quiet_NaN();
    expect(!triwhirl::validFuzzyBalanceConfig(invalid),
           "non-finite singleton is rejected", failures);

    invalid = config;
    invalid.target_velocity_singletons[0] = 1.01F;
    expect(!triwhirl::validFuzzyBalanceConfig(invalid),
           "out-of-range singleton is rejected", failures);
  }

  if (failures == 0) {
    std::cout << "PASS fuzzy balance controller boundary contract\n";
    return 0;
  }
  std::cerr << "FAIL fuzzy balance controller boundary contract failures="
            << failures << '\n';
  return 1;
}
