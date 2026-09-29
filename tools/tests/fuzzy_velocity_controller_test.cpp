#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

#include "triwhirl/fuzzy_velocity_controller.hpp"

namespace {

constexpr float kNominalWheelA = -7.8475F;
constexpr float kNominalWheelB = 202.2085F;

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

triwhirl::FuzzyVelocityControllerConfig sitlConfig() {
  triwhirl::FuzzyVelocityControllerConfig config{};
  // Calibrate only the fuzzy hold singleton endpoints from the existing
  // provisional B/C-midpoint local wheel submodel:
  //   wheel_dot = A22*wheel + B2*Vq, body states held at zero.
  // This is SITL evidence, not hardware authority.
  config.hold_vq_at_scale_v =
      (-kNominalWheelA / kNominalWheelB) * config.wheel_rate_scale_rad_s;
  return config;
}

struct MotorLoopResult {
  float final_rate_rad_s = 0.0F;
  float max_abs_vq_v = 0.0F;
  bool finite = true;
};

MotorLoopResult runMotorLoop(const float plant_a22, const float plant_b2,
                             const float target_rad_s,
                             const bool inject_kick = false) {
  const auto config = sitlConfig();
  triwhirl::FuzzyVelocityController controller(config);
  constexpr float dt_s = 0.001F;
  float wheel_rate = 0.0F;
  MotorLoopResult result{};
  for (int step = 0; step < 3000; ++step) {
    if (inject_kick && step == 1000) wheel_rate += 5.0F;
    const auto output = controller.step(target_rad_s, wheel_rate, dt_s);
    result.finite = result.finite && output.valid &&
                    std::isfinite(output.vq_v) &&
                    std::isfinite(wheel_rate);
    result.max_abs_vq_v =
        std::max(result.max_abs_vq_v, std::fabs(output.vq_v));
    wheel_rate +=
        dt_s * (plant_a22 * wheel_rate + plant_b2 * output.vq_v);
  }
  result.final_rate_rad_s = wheel_rate;
  return result;
}

}  // namespace

int main() {
  const auto config = sitlConfig();
  require(triwhirl::validFuzzyVelocityControllerConfig(config),
          "SITL fuzzy velocity configuration is valid");

  const auto zero = triwhirl::evaluateFuzzyVelocityController(
      config, 0.0F, 0.0F, 0.0F);
  require(zero.valid && near(zero.vq_v, 0.0F),
          "zero error/rate at stopped wheel commands zero Vq");

  const auto positive = triwhirl::evaluateFuzzyVelocityController(
      config, 15.0F, 0.0F, 0.0F);
  const auto negative = triwhirl::evaluateFuzzyVelocityController(
      config, -15.0F, 0.0F, 0.0F);
  require(positive.valid && positive.vq_v > 0.0F,
          "positive velocity error commands positive Vq");
  require(negative.valid && negative.vq_v < 0.0F,
          "negative velocity error commands negative Vq");
  require(near(positive.vq_v, -negative.vq_v),
          "basic restoring command is mirrored");

  const auto positive_hold = triwhirl::evaluateFuzzyVelocityController(
      config, 0.0F, 0.0F, 20.0F);
  const auto negative_hold = triwhirl::evaluateFuzzyVelocityController(
      config, 0.0F, 0.0F, -20.0F);
  require(positive_hold.valid && positive_hold.hold_vq_v > 0.0F &&
              near(positive_hold.correction_vq_v, 0.0F),
          "fuzzy wheel-rate hold map can sustain positive nonzero speed");
  require(negative_hold.valid && negative_hold.hold_vq_v < 0.0F &&
              near(positive_hold.hold_vq_v, -negative_hold.hold_vq_v),
          "fuzzy steady-speed hold map is mirrored");

  bool domain_valid = true;
  bool limits_respected = true;
  float max_symmetry_error = 0.0F;
  for (int ei = -40; ei <= 40; ei += 2) {
    for (int di = -400; di <= 400; di += 40) {
      for (int wi = -80; wi <= 80; wi += 10) {
        const float error = static_cast<float>(ei);
        const float error_rate = static_cast<float>(di);
        const float wheel_rate = static_cast<float>(wi);
        const auto a = triwhirl::evaluateFuzzyVelocityController(
            config, error, error_rate, wheel_rate);
        const auto b = triwhirl::evaluateFuzzyVelocityController(
            config, -error, -error_rate, -wheel_rate);
        domain_valid = domain_valid && a.valid && b.valid;
        limits_respected = limits_respected &&
                           std::fabs(a.vq_v) <= config.vq_limit_v + 1.0e-6F &&
                           std::fabs(b.vq_v) <= config.vq_limit_v + 1.0e-6F;
        max_symmetry_error =
            std::max(max_symmetry_error, std::fabs(a.vq_v + b.vq_v));
      }
    }
  }
  require(domain_valid, "composite fuzzy surface remains finite on domain sweep");
  require(limits_respected, "composite fuzzy surface respects hard Vq limit");
  require(max_symmetry_error < 1.0e-5F,
          "composite fuzzy motor surface preserves bilateral antisymmetry");

  bool boundary_sweep_valid = true;
  float max_adjacent_jump = 0.0F;
  auto previous = triwhirl::evaluateFuzzyVelocityController(
      config, -config.error_scale_rad_s, 0.0F, 0.0F);
  for (int step = 1; step <= 600; ++step) {
    const float error = -config.error_scale_rad_s +
                        2.0F * config.error_scale_rad_s *
                            static_cast<float>(step) / 600.0F;
    const auto current = triwhirl::evaluateFuzzyVelocityController(
        config, error, 0.0F, 0.0F);
    boundary_sweep_valid = boundary_sweep_valid && current.valid;
    max_adjacent_jump =
        std::max(max_adjacent_jump, std::fabs(current.vq_v - previous.vq_v));
    previous = current;
  }
  require(boundary_sweep_valid, "membership-boundary sweep remains finite");
  require(max_adjacent_jump < 0.02F,
          "fuzzy correction is continuous across membership boundaries");

  const auto clamped = triwhirl::evaluateFuzzyVelocityController(
      config, 10.0F * config.error_scale_rad_s,
      10.0F * config.error_rate_scale_rad_s2,
      10.0F * config.wheel_rate_scale_rad_s);
  require(clamped.valid && clamped.input_clamped,
          "out-of-universe inputs use end shoulders deterministically");
  require(std::fabs(clamped.vq_v) <= config.vq_limit_v,
          "extreme fuzzy request remains hard-clamped");

  const auto invalid = triwhirl::evaluateFuzzyVelocityController(
      config, std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F);
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

  // Motor-loop SITL uses only the measured wheel sub-dynamics of the existing
  // provisional local B/C models with body angle/rate held at zero. One fixed
  // nominal fuzzy rule calibration is tested against B, C, and midpoint plants.
  constexpr float b_a22 = -6.451F;
  constexpr float b_b2 = 181.134F;
  constexpr float c_a22 = -9.244F;
  constexpr float c_b2 = 223.283F;

  for (const float target : {20.0F, -20.0F}) {
    const auto nominal =
        runMotorLoop(kNominalWheelA, kNominalWheelB, target);
    const auto plant_b = runMotorLoop(b_a22, b_b2, target);
    const auto plant_c = runMotorLoop(c_a22, c_b2, target);
    require(nominal.finite && plant_b.finite && plant_c.finite,
            "local B/C motor-loop SITL remains finite");
    require(std::fabs(nominal.final_rate_rad_s - target) < 0.05F,
            "nominal fuzzy motor-loop SITL reaches target");
    require(std::fabs(plant_b.final_rate_rad_s - target) < 1.1F &&
                std::fabs(plant_c.final_rate_rad_s - target) < 1.1F,
            "one nominal fuzzy calibration tolerates B/C wheel-model spread");
    require(nominal.max_abs_vq_v <= config.vq_limit_v &&
                plant_b.max_abs_vq_v <= config.vq_limit_v &&
                plant_c.max_abs_vq_v <= config.vq_limit_v,
            "motor-loop SITL never exceeds hard Vq authority");
  }

  const auto disturbed =
      runMotorLoop(kNominalWheelA, kNominalWheelB, 20.0F, true);
  require(disturbed.finite &&
              std::fabs(disturbed.final_rate_rad_s - 20.0F) < 0.05F,
          "nominal fuzzy motor loop recovers from +5 rad/s wheel kick");

  std::cout << "PASS pure-fuzzy motor velocity foundation and local SITL\n";
  return 0;
}
