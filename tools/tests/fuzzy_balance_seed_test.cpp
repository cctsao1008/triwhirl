#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>

#include "triwhirl/fuzzy_balance_controller.hpp"
#include "triwhirl/fuzzy_balance_seed.hpp"
#include "triwhirl/upright_geometry.hpp"

namespace {

constexpr float kWheelScaleRadS = 0.4F;
constexpr float kTargetLimitRadS = 1.0F;
constexpr float kWheelToTargetRatio = kWheelScaleRadS / kTargetLimitRadS;

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

std::size_t ruleIndex(const std::size_t theta, const std::size_t rate,
                      const std::size_t wheel) {
  return (theta * triwhirl::fuzzy::kFiveTermCount + rate) *
             triwhirl::fuzzy::kFiveTermCount +
         wheel;
}

triwhirl::FuzzyBalanceController seedController() {
  triwhirl::FuzzyBalanceConfig config{};
  config.theta_error_scale_rad = 1.0F;
  config.theta_rate_scale_rad_s = 1.0F;
  config.wheel_velocity_scale_rad_s = kWheelScaleRadS;
  config.target_velocity_limit_rad_s = kTargetLimitRadS;
  config.target_velocity_singletons =
      triwhirl::fuzzy_balance::makeQualitativeRuleSeed(
          config.wheel_velocity_scale_rad_s,
          config.target_velocity_limit_rad_s);
  config.rule_surface_configured = true;
  return triwhirl::FuzzyBalanceController(config);
}

}  // namespace

int main() {
  int failures = 0;
  const auto seed = triwhirl::fuzzy_balance::makeQualitativeRuleSeed(
      kWheelScaleRadS, kTargetLimitRadS);

  bool bounded = true;
  bool odd = true;
  for (std::size_t i = 0; i < triwhirl::fuzzy::kFiveTermCount; ++i) {
    for (std::size_t j = 0; j < triwhirl::fuzzy::kFiveTermCount; ++j) {
      for (std::size_t k = 0; k < triwhirl::fuzzy::kFiveTermCount; ++k) {
        const float value = seed[ruleIndex(i, j, k)];
        bounded = bounded && std::isfinite(value) && value >= -1.0F &&
                  value <= 1.0F;
        const float mirror = seed[ruleIndex(
            triwhirl::fuzzy::kFiveTermCount - 1 - i,
            triwhirl::fuzzy::kFiveTermCount - 1 - j,
            triwhirl::fuzzy::kFiveTermCount - 1 - k)];
        odd = odd && near(value, -mirror);
      }
    }
  }
  expect(bounded, "all 125 qualitative singletons are finite and bounded",
         failures);
  expect(odd, "singleton surface has exact mirrored-state odd symmetry",
         failures);

  expect(near(seed[ruleIndex(2, 2, 2)], 0.0F),
         "ZE/ZE/ZE maps to zero target", failures);

  bool wheel_baseline = true;
  for (std::size_t wheel = 0; wheel < triwhirl::fuzzy::kFiveTermCount;
       ++wheel) {
    const float expected =
        triwhirl::fuzzy::kFiveTermCenters[wheel] * kWheelToTargetRatio;
    wheel_baseline = wheel_baseline &&
                     near(seed[ruleIndex(2, 2, wheel)], expected);
  }
  expect(wheel_baseline,
         "ZE/ZE/wheel converts the wheel center into the physical target scale",
         failures);

  expect(seed[ruleIndex(4, 2, 2)] < seed[ruleIndex(2, 2, 2)] &&
             seed[ruleIndex(0, 2, 2)] > seed[ruleIndex(2, 2, 2)],
         "body-angle terms shift target in the restoring direction", failures);
  expect(seed[ruleIndex(2, 4, 2)] < seed[ruleIndex(2, 2, 2)] &&
             seed[ruleIndex(2, 0, 2)] > seed[ruleIndex(2, 2, 2)],
         "body-rate terms shift target in the damping direction", failures);

  expect(seed[ruleIndex(3, 2, 3)] < seed[ruleIndex(2, 2, 3)] &&
             seed[ruleIndex(1, 2, 3)] > seed[ruleIndex(2, 2, 3)],
         "attitude correction is relative to a nonzero wheel baseline",
         failures);

  const auto controller = seedController();
  expect(controller.valid(), "qualitative seed forms a valid fuzzy controller",
         failures);

  {
    constexpr float wheel_velocity = 0.25F;
    const auto output = controller.evaluate(
        triwhirl::FuzzyBalanceInput{0.0F, 0.0F, wheel_velocity});
    expect(output.valid &&
               near(output.target_velocity_rad_s, wheel_velocity) &&
               near(output.target_velocity_normalized,
                    wheel_velocity / kTargetLimitRadS),
           "zero attitude urgency preserves wheel rad/s even when input/output scales differ",
           failures);
  }

  {
    const auto baseline =
        controller.evaluate(triwhirl::FuzzyBalanceInput{0.0F, 0.0F, 0.25F});
    const auto restoring =
        controller.evaluate(triwhirl::FuzzyBalanceInput{0.25F, 0.0F, 0.25F});
    const auto damping =
        controller.evaluate(triwhirl::FuzzyBalanceInput{0.0F, 0.25F, 0.25F});
    expect(baseline.valid && restoring.valid && damping.valid &&
               restoring.target_velocity_normalized <
                   baseline.target_velocity_normalized &&
               damping.target_velocity_normalized <
                   baseline.target_velocity_normalized,
           "restoring and damping intent shift target below a positive wheel baseline",
           failures);
  }

  {
    const auto positive =
        controller.evaluate(triwhirl::FuzzyBalanceInput{0.25F, 0.15F, 0.10F});
    const auto negative = controller.evaluate(
        triwhirl::FuzzyBalanceInput{-0.25F, -0.15F, -0.10F});
    expect(positive.valid && negative.valid &&
               near(positive.target_velocity_normalized,
                    -negative.target_velocity_normalized),
           "continuous Sugeno inference preserves mirrored-state symmetry",
           failures);
  }

  {
    const auto left = controller.evaluate(
        triwhirl::FuzzyBalanceInput{0.5F - 1.0e-4F, 0.0F, 0.0F});
    const auto right = controller.evaluate(
        triwhirl::FuzzyBalanceInput{0.5F + 1.0e-4F, 0.0F, 0.0F});
    expect(left.valid && right.valid &&
               std::fabs(left.target_velocity_normalized -
                         right.target_velocity_normalized) < 1.0e-3F,
           "rule interpolation remains continuous across a membership center",
           failures);
  }

  {
    constexpr float reference = 1.1F;
    constexpr float delta = 0.12F;
    const float error_a =
        triwhirl::periodicUprightErrorRad(reference + delta, reference);
    const float error_b = triwhirl::periodicUprightErrorRad(
        reference + triwhirl::kUprightPeriodRad + delta, reference);
    const float error_c = triwhirl::periodicUprightErrorRad(
        reference - triwhirl::kUprightPeriodRad + delta, reference);

    const auto out_a = controller.evaluate(
        triwhirl::FuzzyBalanceInput{error_a, 0.2F, -0.1F});
    const auto out_b = controller.evaluate(
        triwhirl::FuzzyBalanceInput{error_b, 0.2F, -0.1F});
    const auto out_c = controller.evaluate(
        triwhirl::FuzzyBalanceInput{error_c, 0.2F, -0.1F});

    expect(out_a.valid && out_b.valid && out_c.valid &&
               near(error_a, error_b) && near(error_a, error_c) &&
               near(out_a.target_velocity_normalized,
                    out_b.target_velocity_normalized) &&
               near(out_a.target_velocity_normalized,
                    out_c.target_velocity_normalized),
           "A/B/C 120-degree uprights share one fuzzy control coordinate",
           failures);
  }

  {
    const auto invalid = triwhirl::fuzzy_balance::makeQualitativeRuleSeed(
        1.0F, 0.0F);
    expect(!std::isfinite(invalid[0]),
           "invalid scale ratio produces a fail-closed invalid rule seed",
           failures);
  }

  if (failures == 0) {
    std::cout << "PASS pure-fuzzy balance seed contract\n";
    return 0;
  }
  std::cerr << "FAIL pure-fuzzy balance seed contract failures=" << failures
            << '\n';
  return 1;
}
