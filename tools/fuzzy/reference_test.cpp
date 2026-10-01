#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

#include <fl/Headers.h>

#include "triwhirl/fuzzy.hpp"

namespace {

constexpr float kTolerance = 1.0e-5F;

void require(const bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL " << message << '\n';
    std::exit(1);
  }
}

bool near(const float a, const float b, const float tolerance = kTolerance) {
  return std::fabs(a - b) <= tolerance;
}

std::array<float, triwhirl::fuzzy::kFiveTermRuleCount3D>
makeReferenceSingletons() {
  std::array<float, triwhirl::fuzzy::kFiveTermRuleCount3D> result{};
  for (std::size_t i = 0; i < triwhirl::fuzzy::kFiveTermCount; ++i) {
    for (std::size_t j = 0; j < triwhirl::fuzzy::kFiveTermCount; ++j) {
      for (std::size_t k = 0; k < triwhirl::fuzzy::kFiveTermCount; ++k) {
        const std::size_t index =
            (i * triwhirl::fuzzy::kFiveTermCount + j) *
                triwhirl::fuzzy::kFiveTermCount +
            k;
        const int level = 6 - static_cast<int>(i + j + k);
        result[index] = static_cast<float>(level) / 6.0F;
      }
    }
  }
  return result;
}

struct EmbeddedEvaluation {
  bool valid = false;
  float value = 0.0F;
  float weight_sum = 0.0F;
};

EmbeddedEvaluation embeddedEvaluate(
    const float theta, const float theta_dot, const float wheel,
    const std::array<float, triwhirl::fuzzy::kFiveTermRuleCount3D>&
        singletons) {
  const auto result = triwhirl::fuzzy::evaluateSugeno3D(
      triwhirl::fuzzy::fiveTermMembership(theta),
      triwhirl::fuzzy::fiveTermMembership(theta_dot),
      triwhirl::fuzzy::fiveTermMembership(wheel), singletons);

  EmbeddedEvaluation evaluation{};
  evaluation.valid = result.valid;
  evaluation.value = result.value;
  evaluation.weight_sum = result.weight_sum;
  return evaluation;
}

class ReferenceEngine {
 public:
  explicit ReferenceEngine(const std::string& path)
      : engine_(fl::FllImporter().fromFile(path)) {
    require(engine_ != nullptr, "FuzzyLite importer returned an engine");

    std::string status;
    require(engine_->isReady(&status), status.empty() ? "FuzzyLite engine ready"
                                                      : status.c_str());

    theta_ = engine_->getInputVariable("theta_error");
    theta_dot_ = engine_->getInputVariable("theta_dot");
    wheel_ = engine_->getInputVariable("wheel_velocity");
    target_ = engine_->getOutputVariable("target_velocity");
    require(theta_ != nullptr && theta_dot_ != nullptr && wheel_ != nullptr &&
                target_ != nullptr,
            "FuzzyLite variables resolved");
  }

  float evaluate(const float theta, const float theta_dot, const float wheel) {
    theta_->setValue(static_cast<fl::scalar>(theta));
    theta_dot_->setValue(static_cast<fl::scalar>(theta_dot));
    wheel_->setValue(static_cast<fl::scalar>(wheel));
    engine_->process();
    return static_cast<float>(target_->getValue());
  }

 private:
  std::unique_ptr<fl::Engine> engine_;
  fl::InputVariable* theta_ = nullptr;
  fl::InputVariable* theta_dot_ = nullptr;
  fl::InputVariable* wheel_ = nullptr;
  fl::OutputVariable* target_ = nullptr;
};

float clampNormalized(const float value) {
  return std::clamp(value, -1.0F, 1.0F);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: fuzzy_reference_test <balance.fll>\n";
    return 2;
  }

  ReferenceEngine reference(argv[1]);
  const auto singletons = makeReferenceSingletons();

  std::size_t center_count = 0;
  for (std::size_t i = 0; i < triwhirl::fuzzy::kFiveTermCount; ++i) {
    for (std::size_t j = 0; j < triwhirl::fuzzy::kFiveTermCount; ++j) {
      for (std::size_t k = 0; k < triwhirl::fuzzy::kFiveTermCount; ++k) {
        const float theta = triwhirl::fuzzy::kFiveTermCenters[i];
        const float rate = triwhirl::fuzzy::kFiveTermCenters[j];
        const float wheel = triwhirl::fuzzy::kFiveTermCenters[k];
        const float expected =
            static_cast<float>(6 - static_cast<int>(i + j + k)) / 6.0F;

        const auto embedded =
            embeddedEvaluate(theta, rate, wheel, singletons);
        const float oracle = reference.evaluate(theta, rate, wheel);

        require(embedded.valid, "embedded center inference valid");
        require(near(embedded.weight_sum, 1.0F, 1.0e-4F),
                "embedded center rule weights cover universe");
        require(std::isfinite(oracle), "FuzzyLite center inference finite");
        require(near(embedded.value, expected),
                "embedded center singleton mapping complete");
        require(near(oracle, expected),
                "FuzzyLite center singleton mapping complete");
        ++center_count;
      }
    }
  }
  require(center_count == triwhirl::fuzzy::kFiveTermRuleCount3D,
          "all 125 linguistic centers covered");

  double absolute_error_sum = 0.0;
  float max_absolute_error = 0.0F;
  float max_antisymmetry_error = 0.0F;
  std::size_t sample_count = 0;

  for (int i = 0; i <= 20; ++i) {
    const float theta = -1.0F + static_cast<float>(i) / 10.0F;
    for (int j = 0; j <= 20; ++j) {
      const float rate = -1.0F + static_cast<float>(j) / 10.0F;
      for (int k = 0; k <= 20; ++k) {
        const float wheel = -1.0F + static_cast<float>(k) / 10.0F;

        const auto embedded =
            embeddedEvaluate(theta, rate, wheel, singletons);
        const float oracle = reference.evaluate(theta, rate, wheel);
        require(embedded.valid, "embedded dense-grid inference valid");
        require(near(embedded.weight_sum, 1.0F, 1.0e-4F),
                "embedded dense-grid rule weights cover universe");
        require(std::isfinite(oracle), "FuzzyLite dense-grid output finite");
        require(embedded.value >= -1.00001F && embedded.value <= 1.00001F,
                "embedded dense-grid output bounded");
        require(oracle >= -1.00001F && oracle <= 1.00001F,
                "FuzzyLite dense-grid output bounded");

        const float error = std::fabs(embedded.value - oracle);
        max_absolute_error = std::max(max_absolute_error, error);
        absolute_error_sum += error;

        const float mirrored = reference.evaluate(-theta, -rate, -wheel);
        require(std::isfinite(mirrored), "FuzzyLite mirrored output finite");
        max_antisymmetry_error =
            std::max(max_antisymmetry_error, std::fabs(oracle + mirrored));
        ++sample_count;
      }
    }
  }

  struct OutsideSample {
    float theta;
    float rate;
    float wheel;
  };
  constexpr std::array<OutsideSample, 4> outside{{
      {1.25F, 0.2F, -0.3F},
      {-1.4F, 0.8F, 1.3F},
      {0.4F, -1.6F, 0.1F},
      {2.0F, -2.0F, 2.0F},
  }};

  for (const auto& sample : outside) {
    const auto embedded = embeddedEvaluate(sample.theta, sample.rate,
                                           sample.wheel, singletons);
    const float oracle =
        reference.evaluate(sample.theta, sample.rate, sample.wheel);
    const float oracle_clamped = reference.evaluate(
        clampNormalized(sample.theta), clampNormalized(sample.rate),
        clampNormalized(sample.wheel));

    require(embedded.valid, "embedded out-of-domain inference valid");
    require(std::isfinite(oracle), "FuzzyLite out-of-domain output finite");
    require(near(oracle, oracle_clamped),
            "FuzzyLite lock-range matches explicit normalized clamp");
    require(near(embedded.value, oracle),
            "embedded clamp matches FuzzyLite lock-range");
  }

  const double mean_absolute_error =
      sample_count == 0 ? std::numeric_limits<double>::quiet_NaN()
                        : absolute_error_sum / static_cast<double>(sample_count);

  require(max_absolute_error <= kTolerance,
          "dense-grid embedded/FuzzyLite parity within tolerance");
  require(max_antisymmetry_error <= kTolerance,
          "reference surface is antisymmetric within tolerance");

  std::cout << std::setprecision(9)
            << "PASS fuzzy reference parity samples=" << sample_count
            << " centers=" << center_count
            << " max_abs_error=" << max_absolute_error
            << " mean_abs_error=" << mean_absolute_error
            << " max_antisymmetry_error=" << max_antisymmetry_error << '\n';
  std::cout << "NOTE structural normalized parity baseline only; not a tuned "
               "or hardware-ready balance controller\n";
  return 0;
}
