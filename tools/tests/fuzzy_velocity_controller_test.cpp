#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "triwhirl/fuzzy_velocity_controller.hpp"

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

}  // namespace

int main() {
  const triwhirl::FuzzyVelocityControllerConfig config{};
  require(triwhirl::validFuzzyVelocityControllerConfig(config),
          "default fuzzy velocity configuration is valid");

  const auto zero =
      triwhirl::evaluateFuzzyVelocityController(config, 0.0F, 0.0F);
  require(zero.valid && near(zero.vq_v, 0.0F),
          "zero error and zero error-rate command zero Vq");

  const auto positive =
      triwhirl::evaluateFuzzyVelocityController(config, 15.0F, 0.0F);
  const auto negative =
      triwhirl::evaluateFuzzyVelocityController(config, -15.0F, 0.0F);
  require(positive.valid && positive.vq_v > 0.0F,
          "positive velocity error commands positive Vq");
  require(negative.valid && negative.vq_v < 0.0F,
          "negative velocity error commands negative Vq");
  require(near(positive.vq_v, -negative.vq_v),
          "basic restoring command is mirrored");

  bool domain_valid = true;
  bool limits_respected = true;
  float max_symmetry_error = 0.0F;
  for (int ei = -40; ei <= 40; ++ei) {
    for (int di = -400; di <= 400; di += 20) {
      const float error = static_cast<float>(ei);
      const float error_rate = static_cast<float>(di);
      const auto a = triwhirl::evaluateFuzzyVelocityController(
          config, error, error_rate);
      const auto b = triwhirl::evaluateFuzzyVelocityController(
          config, -error, -error_rate);
      domain_valid = domain_valid && a.valid && b.valid;
      limits_respected = limits_respected &&
                         std::fabs(a.vq_v) <= config.vq_limit_v + 1.0e-6F &&
                         std::fabs(b.vq_v) <= config.vq_limit_v + 1.0e-6F;
      max_symmetry_error =
          std::max(max_symmetry_error, std::fabs(a.vq_v + b.vq_v));
    }
  }
  require(domain_valid, "fuzzy surface remains finite on the swept domain");
  require(limits_respected, "fuzzy surface respects the hard Vq limit");
  require(max_symmetry_error < 1.0e-5F,
          "fuzzy rule surface preserves bilateral antisymmetry");

  bool boundary_sweep_valid = true;
  float max_adjacent_jump = 0.0F;
  auto previous = triwhirl::evaluateFuzzyVelocityController(
      config, -config.error_scale_rad_s, 0.0F);
  for (int step = 1; step <= 600; ++step) {
    const float error = -config.error_scale_rad_s +
                        2.0F * config.error_scale_rad_s *
                            static_cast<float>(step) / 600.0F;
    const auto current =
        triwhirl::evaluateFuzzyVelocityController(config, error, 0.0F);
    boundary_sweep_valid = boundary_sweep_valid && current.valid;
    max_adjacent_jump =
        std::max(max_adjacent_jump, std::fabs(current.vq_v - previous.vq_v));
    previous = current;
  }
  require(boundary_sweep_valid,
          "membership-boundary sweep remains finite");
  require(max_adjacent_jump < 0.03F,
          "fuzzy output is continuous across membership boundaries");

  const auto clamped = triwhirl::evaluateFuzzyVelocityController(
      config, 10.0F * config.error_scale_rad_s,
      10.0F * config.error_rate_scale_rad_s2);
  require(clamped.valid && clamped.input_clamped,
          "out-of-universe inputs use end shoulders deterministically");
  require(near(clamped.vq_v, config.vq_limit_v),
          "positive extreme input reaches bounded positive authority");

  const auto invalid = triwhirl::evaluateFuzzyVelocityController(
      config, std::numeric_limits<float>::quiet_NaN(), 0.0F);
  require(!invalid.valid, "non-finite fuzzy input is rejected");

  triwhirl::FuzzyVelocityController controller(config);
  const auto first = controller.step(10.0F, 0.0F, 0.001F);
  require(first.valid && near(first.error_rate_rad_s2, 0.0F),
          "first stateful sample does not synthesize derivative kick");
  const auto second = controller.step(10.0F, 2.0F, 0.001F);
  require(second.valid && second.error_rate_rad_s2 < 0.0F,
          "stateful fuzzy controller observes improving velocity error");
  controller.reset();
  const auto reset = controller.step(-10.0F, 0.0F, 0.001F);
  require(reset.valid && near(reset.error_rate_rad_s2, 0.0F),
          "reset clears fuzzy velocity history");

  std::cout << "PASS pure-fuzzy motor velocity foundation\n";
  return 0;
}
