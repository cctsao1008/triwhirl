#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "triwhirl/standup_commissioning.hpp"
#include "triwhirl/standup_controller.hpp"
#include "triwhirl/upright_geometry.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kThetaReferenceRad = 68.0 * kPi / 180.0;
constexpr std::uint32_t kControlPeriodUs = 1000U;
constexpr double kControlPeriodS = 0.001;
constexpr double kPlantStepS = 0.0001;
constexpr int kPlantSubsteps = 10;
constexpr std::uint32_t kStartTimeUs = 1000000U;
constexpr double kLongRunGateDurationS = 10.0;

struct State {
  double theta_error_rad = 0.0;
  double theta_rate_rad_s = 0.0;
  double wheel_rate_rad_s = 0.0;
  double wheel_angle_rad = 0.0;
};

struct PlantModel {
  std::string name;
  std::array<std::array<double, 3>, 3> a{};
  std::array<double, 3> b{};
};

PlantModel provisionalB() {
  PlantModel model{};
  model.name = "swing-native-04-B-current-vq";
  model.a = {{{0.0, 1.0, 0.0},
              {149.234, 4.205, -0.871},
              {-111.267, 11.347, -6.451}}};
  // The historical fit used the earlier Vq coordinate. The input column is
  // sign-normalized to the current direction-check convention. This remains a
  // provisional local commissioning model, not a validated digital twin.
  model.b = {{0.0, 5.099, 181.134}};
  return model;
}

PlantModel provisionalC() {
  PlantModel model{};
  model.name = "swing-native-04-C-current-vq";
  model.a = {{{0.0, 1.0, 0.0},
              {155.128, 8.850, -2.329},
              {-317.952, 22.223, -9.244}}};
  model.b = {{0.0, 31.421, 223.283}};
  return model;
}

PlantModel provisionalNominal() {
  const PlantModel b = provisionalB();
  const PlantModel c = provisionalC();
  PlantModel model{};
  model.name = "swing-native-04-BC-midpoint-current-vq";
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      model.a[row][col] = 0.5 * (b.a[row][col] + c.a[row][col]);
    }
    model.b[row] = 0.5 * (b.b[row] + c.b[row]);
  }
  return model;
}

PlantModel plantByName(const std::string& name) {
  if (name == "B" || name == "b") return provisionalB();
  if (name == "C" || name == "c") return provisionalC();
  if (name == "nominal" || name == "BC" || name == "bc") {
    return provisionalNominal();
  }
  throw std::runtime_error("unknown plant profile: " + name);
}

State derivative(const PlantModel& plant, const State& state,
                 const double vq_v) {
  const std::array<double, 3> x{{state.theta_error_rad,
                                 state.theta_rate_rad_s,
                                 state.wheel_rate_rad_s}};
  std::array<double, 3> dx{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      dx[row] += plant.a[row][col] * x[col];
    }
    dx[row] += plant.b[row] * vq_v;
  }
  return State{dx[0], dx[1], dx[2], state.wheel_rate_rad_s};
}

State addScaled(const State& a, const State& b, const double scale) {
  return State{a.theta_error_rad + scale * b.theta_error_rad,
               a.theta_rate_rad_s + scale * b.theta_rate_rad_s,
               a.wheel_rate_rad_s + scale * b.wheel_rate_rad_s,
               a.wheel_angle_rad + scale * b.wheel_angle_rad};
}

State rk4Step(const PlantModel& plant, const State& state, const double vq_v,
              const double dt_s) {
  const State k1 = derivative(plant, state, vq_v);
  const State k2 = derivative(plant, addScaled(state, k1, 0.5 * dt_s), vq_v);
  const State k3 = derivative(plant, addScaled(state, k2, 0.5 * dt_s), vq_v);
  const State k4 = derivative(plant, addScaled(state, k3, dt_s), vq_v);
  State next = state;
  next.theta_error_rad +=
      dt_s * (k1.theta_error_rad + 2.0 * k2.theta_error_rad +
              2.0 * k3.theta_error_rad + k4.theta_error_rad) /
      6.0;
  next.theta_rate_rad_s +=
      dt_s * (k1.theta_rate_rad_s + 2.0 * k2.theta_rate_rad_s +
              2.0 * k3.theta_rate_rad_s + k4.theta_rate_rad_s) /
      6.0;
  next.wheel_rate_rad_s +=
      dt_s * (k1.wheel_rate_rad_s + 2.0 * k2.wheel_rate_rad_s +
              2.0 * k3.wheel_rate_rad_s + k4.wheel_rate_rad_s) /
      6.0;
  next.wheel_angle_rad +=
      dt_s * (k1.wheel_angle_rad + 2.0 * k2.wheel_angle_rad +
              2.0 * k3.wheel_angle_rad + k4.wheel_angle_rad) /
      6.0;
  return next;
}

triwhirl::StandupControllerInput controllerInput(const std::uint32_t now_us,
                                                 const State& state) {
  triwhirl::StandupControllerInput input{};
  input.now_us = now_us;
  input.theta_rad =
      static_cast<float>(kThetaReferenceRad + state.theta_error_rad);
  input.theta_rate_rad_s = static_cast<float>(state.theta_rate_rad_s);
  input.wheel_rate_rad_s = static_cast<float>(state.wheel_rate_rad_s);
  input.valid = true;
  return input;
}

struct EvidenceSample {
  double t_s = 0.0;
  State state{};
  triwhirl::StandupControllerOutput output{};
};

std::vector<EvidenceSample> runClosedLoop(const PlantModel& plant,
                                          const State& initial,
                                          const double duration_s) {
  if (!(duration_s > 0.0)) {
    throw std::runtime_error("duration must be positive");
  }
  const auto config = triwhirl::makeStandupCommissioningConfig(
      static_cast<float>(kThetaReferenceRad), 4.0F);
  triwhirl::StandupController controller(config);
  State state = initial;
  std::uint32_t now_us = kStartTimeUs;
  auto input = controllerInput(now_us, state);
  controller.reset(input);
  auto output = controller.update(input);

  const int control_steps =
      static_cast<int>(std::llround(duration_s / kControlPeriodS));
  std::vector<EvidenceSample> evidence;
  evidence.reserve(static_cast<std::size_t>(control_steps) + 1U);

  for (int step = 0; step <= control_steps; ++step) {
    evidence.push_back(EvidenceSample{
        static_cast<double>(step) * kControlPeriodS, state, output});
    if (step == control_steps) break;

    for (int substep = 0; substep < kPlantSubsteps; ++substep) {
      state = rk4Step(plant, state, static_cast<double>(output.vq_v),
                      kPlantStepS);
    }
    now_us += kControlPeriodUs;
    input = controllerInput(now_us, state);
    output = controller.update(input);
    if (!output.valid || !std::isfinite(output.vq_v) ||
        !std::isfinite(state.theta_error_rad) ||
        !std::isfinite(state.theta_rate_rad_s) ||
        !std::isfinite(state.wheel_rate_rad_s) ||
        !std::isfinite(state.wheel_angle_rad)) {
      throw std::runtime_error("non-finite closed-loop state/output");
    }
  }
  return evidence;
}

const char* boolText(const bool value) { return value ? "1" : "0"; }

void writeCsv(const std::string& path,
              const std::vector<EvidenceSample>& evidence) {
  std::ofstream stream(path);
  if (!stream) throw std::runtime_error("cannot open output: " + path);
  stream << "t_s,true_error_rad,true_error_deg,true_theta_rate_rad_s,"
            "true_wheel_rate_rad_s,true_wheel_angle_rad,phase,settling,stable,"
            "filtered_rate_rad_s,target_velocity_rad_s,velocity_error_rad_s,"
            "velocity_integral_v,vq_unclamped_v,vq_target_v,vq_applied_v,"
            "target_saturated,vq_saturated\n";
  stream << std::setprecision(10);
  for (const auto& sample : evidence) {
    const auto& state = sample.state;
    const auto& output = sample.output;
    stream << sample.t_s << ',' << state.theta_error_rad << ','
           << state.theta_error_rad * kRadToDeg << ','
           << state.theta_rate_rad_s << ',' << state.wheel_rate_rad_s << ','
           << state.wheel_angle_rad << ','
           << triwhirl::standupPhaseName(output.phase) << ','
           << boolText(output.settling) << ',' << boolText(output.stable) << ','
           << output.filtered_rate_rad_s << ',' << output.target_velocity_rad_s
           << ',' << output.velocity_error_rad_s << ','
           << output.velocity_integral_v << ',' << output.vq_unclamped_v << ','
           << output.vq_target_v << ',' << output.vq_v << ','
           << boolText(output.target_saturated) << ','
           << boolText(output.vq_saturated) << '\n';
  }
}

struct RunMetrics {
  double max_abs_error_deg = 0.0;
  double max_abs_rate_rad_s = 0.0;
  double max_abs_wheel_rad_s = 0.0;
  double max_abs_vq_v = 0.0;
  double tail_max_abs_error_deg = 0.0;
  int zero_crossings = 0;
  int vq_sign_changes = 0;
  bool settling_seen = false;
  bool stable_seen = false;
  bool all_balance = true;
  bool any_target_saturation = false;
  bool any_vq_saturation = false;
};

int signWithDeadband(const double value, const double deadband) {
  if (value > deadband) return 1;
  if (value < -deadband) return -1;
  return 0;
}

RunMetrics summarize(const std::vector<EvidenceSample>& evidence) {
  RunMetrics metrics{};
  int previous_vq_sign = 0;
  const double end_t = evidence.empty() ? 0.0 : evidence.back().t_s;
  const double tail_start = std::max(0.0, end_t - 2.0);
  for (std::size_t i = 0; i < evidence.size(); ++i) {
    const auto& sample = evidence[i];
    const double abs_error_deg =
        std::fabs(sample.state.theta_error_rad) * kRadToDeg;
    metrics.max_abs_error_deg = std::max(metrics.max_abs_error_deg,
                                         abs_error_deg);
    if (sample.t_s >= tail_start) {
      metrics.tail_max_abs_error_deg =
          std::max(metrics.tail_max_abs_error_deg, abs_error_deg);
    }
    metrics.max_abs_rate_rad_s =
        std::max(metrics.max_abs_rate_rad_s,
                 std::fabs(sample.state.theta_rate_rad_s));
    metrics.max_abs_wheel_rad_s =
        std::max(metrics.max_abs_wheel_rad_s,
                 std::fabs(sample.state.wheel_rate_rad_s));
    metrics.max_abs_vq_v =
        std::max(metrics.max_abs_vq_v,
                 std::fabs(static_cast<double>(sample.output.vq_v)));
    metrics.settling_seen = metrics.settling_seen || sample.output.settling;
    metrics.stable_seen = metrics.stable_seen || sample.output.stable;
    metrics.all_balance =
        metrics.all_balance &&
        sample.output.phase == triwhirl::StandupPhase::kBalance;
    metrics.any_target_saturation =
        metrics.any_target_saturation || sample.output.target_saturated;
    metrics.any_vq_saturation =
        metrics.any_vq_saturation || sample.output.vq_saturated;

    if (i > 0U) {
      const double previous_error = evidence[i - 1U].state.theta_error_rad;
      const double current_error = sample.state.theta_error_rad;
      if ((previous_error > 0.0 && current_error <= 0.0) ||
          (previous_error < 0.0 && current_error >= 0.0)) {
        ++metrics.zero_crossings;
      }
    }

    const int current_vq_sign = signWithDeadband(sample.output.vq_v, 1.0e-4);
    if (current_vq_sign != 0) {
      if (previous_vq_sign != 0 && current_vq_sign != previous_vq_sign) {
        ++metrics.vq_sign_changes;
      }
      previous_vq_sign = current_vq_sign;
    }
  }
  return metrics;
}

bool longRunBalanceGatePass(const std::vector<EvidenceSample>& evidence,
                            const RunMetrics& metrics) {
  if (evidence.empty() || evidence.back().t_s + 1.0e-9 < kLongRunGateDurationS) {
    return false;
  }
  return metrics.all_balance && metrics.settling_seen && metrics.stable_seen &&
         metrics.zero_crossings >= 1 && metrics.vq_sign_changes >= 1 &&
         metrics.max_abs_error_deg < 5.0 &&
         metrics.tail_max_abs_error_deg < 0.50 &&
         metrics.max_abs_wheel_rad_s < 20.0 &&
         metrics.max_abs_vq_v <= 4.0 && !metrics.any_vq_saturation;
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

triwhirl::StandupControllerInput invariantInput(const std::uint32_t now_us,
                                                const double error_deg,
                                                const double rate_rad_s,
                                                const double wheel_rad_s) {
  return controllerInput(
      now_us, State{error_deg / kRadToDeg, rate_rad_s, wheel_rad_s, 0.0});
}

void checkControllerInvariants(int& failures) {
  const auto production_config = triwhirl::makeStandupCommissioningConfig(
      static_cast<float>(kThetaReferenceRad), 4.0F);

  {
    triwhirl::StandupController positive(production_config);
    auto input = invariantInput(kStartTimeUs, 2.0, 0.0, 0.0);
    positive.reset(input);
    const auto output = positive.update(input);
    expect(output.phase == triwhirl::StandupPhase::kBalance &&
               output.vq_v < 0.0F,
           "positive angle commands restoring Vq", failures);

    triwhirl::StandupController negative(production_config);
    input = invariantInput(kStartTimeUs, -2.0, 0.0, 0.0);
    negative.reset(input);
    const auto mirrored = negative.update(input);
    expect(mirrored.phase == triwhirl::StandupPhase::kBalance &&
               mirrored.vq_v > 0.0F,
           "negative angle commands mirrored restoring Vq", failures);
  }

  {
    triwhirl::StandupController controller(production_config);
    auto input = invariantInput(kStartTimeUs, 0.2, 0.0, -7.0);
    controller.reset(input);
    (void)controller.update(input);
    input = invariantInput(kStartTimeUs + 1000U, -0.2, 0.0, -7.0);
    auto output = controller.update(input);
    expect(output.settling, "true zero crossing acquires settling", failures);
    input = invariantInput(kStartTimeUs + 2000U, 0.16, 0.0, -6.95);
    output = controller.update(input);
    expect(output.settling && output.velocity_error_rad_s < 0.0F &&
               output.vq_v < 0.0F,
           "negative wheel momentum cannot flip positive-error restoring Vq",
           failures);
  }

  {
    triwhirl::StandupController controller(production_config);
    auto input = invariantInput(kStartTimeUs, -0.2, 0.0, 7.0);
    controller.reset(input);
    (void)controller.update(input);
    input = invariantInput(kStartTimeUs + 1000U, 0.2, 0.0, 7.0);
    auto output = controller.update(input);
    expect(output.settling, "mirrored zero crossing acquires settling", failures);
    input = invariantInput(kStartTimeUs + 2000U, -0.16, 0.0, 6.95);
    output = controller.update(input);
    expect(output.settling && output.velocity_error_rad_s > 0.0F &&
               output.vq_v > 0.0F,
           "positive wheel momentum cannot flip negative-error restoring Vq",
           failures);
  }

  {
    auto damping_config = production_config;
    damping_config.lqr_k_angle_unstable = 0.0F;
    damping_config.lqr_k_rate_unstable = 0.0F;
    damping_config.lqr_k_wheel_unstable = 0.0F;
    damping_config.lqr_k_angle_settling = 0.0F;
    damping_config.lqr_k_wheel_settling = 0.0F;
    damping_config.velocity_p_unstable = 0.0F;
    damping_config.velocity_i_unstable = 0.0F;
    damping_config.velocity_p_recovery_unstable = 0.0F;
    damping_config.velocity_i_recovery_unstable = 0.0F;

    triwhirl::StandupController positive_rate(damping_config);
    auto input = invariantInput(kStartTimeUs, 0.1, 0.0, 0.0);
    positive_rate.reset(input);
    (void)positive_rate.update(input);
    input = invariantInput(kStartTimeUs + 1000U, -0.1, 0.0, 0.0);
    (void)positive_rate.update(input);
    input = invariantInput(kStartTimeUs + 2000U, -0.05, 1.0, 0.0);
    auto output = positive_rate.update(input);
    expect(output.settling && output.filtered_rate_rad_s > 0.0F &&
               output.vq_v < 0.0F,
           "positive body rate receives dissipative Vq", failures);

    triwhirl::StandupController negative_rate(damping_config);
    input = invariantInput(kStartTimeUs, -0.1, 0.0, 0.0);
    negative_rate.reset(input);
    (void)negative_rate.update(input);
    input = invariantInput(kStartTimeUs + 1000U, 0.1, 0.0, 0.0);
    (void)negative_rate.update(input);
    input = invariantInput(kStartTimeUs + 2000U, 0.05, -1.0, 0.0);
    output = negative_rate.update(input);
    expect(output.settling && output.filtered_rate_rad_s < 0.0F &&
               output.vq_v > 0.0F,
           "negative body rate receives mirrored dissipative Vq", failures);
  }

  {
    triwhirl::StandupController controller(production_config);
    auto input = invariantInput(kStartTimeUs, -8.0, -1.0, 0.0);
    controller.reset(input);
    auto output = controller.update(input);
    expect(output.phase == triwhirl::StandupPhase::kBalance && !output.settling,
           "8-degree reversal does not falsely acquire settling", failures);
    input = invariantInput(kStartTimeUs + 1000U, -10.5, -1.0, 0.0);
    output = controller.update(input);
    input = invariantInput(kStartTimeUs + 2000U, -12.5, -1.0, 0.0);
    output = controller.update(input);
    expect(output.phase == triwhirl::StandupPhase::kSwingLow && !output.settling,
           "failed approach releases at pre-settling hysteresis", failures);
  }

  {
    triwhirl::StandupController controller(production_config);
    auto input = invariantInput(kStartTimeUs, 2.0, -0.5, 0.0);
    controller.reset(input);
    (void)controller.update(input);
    input = invariantInput(kStartTimeUs + 1000U, -0.2, -0.5, 0.0);
    auto output = controller.update(input);
    expect(output.settling, "settling remains latched after crossing", failures);
    input = invariantInput(kStartTimeUs + 2000U, 30.0, 0.0, 0.0);
    output = controller.update(input);
    expect(output.phase == triwhirl::StandupPhase::kBalance && output.settling,
           "settling continues correction at 30 degrees", failures);
    input = invariantInput(kStartTimeUs + 3000U, 56.0, 0.0, 0.0);
    output = controller.update(input);
    expect(output.phase == triwhirl::StandupPhase::kSwingHigh &&
               !output.settling,
           "settling releases only beyond fall guard", failures);
  }

  {
    triwhirl::StandupController controller(production_config);
    auto input = invariantInput(kStartTimeUs, 1.0, 0.0, 0.0);
    controller.reset(input);
    auto output = controller.update(input);
    for (std::uint32_t step = 1U; step <= 1001U; ++step) {
      input = invariantInput(kStartTimeUs + step * 1000U, 1.0, 0.0, 0.0);
      output = controller.update(input);
    }
    expect(output.stable && output.settling &&
               output.phase == triwhirl::StandupPhase::kBalance,
           "one-second stable qualification is status plus persistent settling",
           failures);
    expect(std::fabs(output.vq_v) > 1.0e-3F,
           "stable qualification does not stop active correction", failures);
  }
}

void checkShortClosedLoop(int& failures) {
  const PlantModel plant = provisionalNominal();
  const auto positive = runClosedLoop(
      plant, State{1.0 / kRadToDeg, -0.2, 0.0, 0.0}, 0.200);
  const auto negative = runClosedLoop(
      plant, State{-1.0 / kRadToDeg, 0.2, 0.0, 0.0}, 0.200);
  const RunMetrics p = summarize(positive);
  const RunMetrics n = summarize(negative);

  expect(p.all_balance && n.all_balance,
         "near-upright closed loop stays in balance mode", failures);
  expect(p.zero_crossings >= 1 && n.zero_crossings >= 1,
         "near-upright closed loop crosses upright in both directions", failures);
  expect(p.settling_seen && n.settling_seen,
         "zero crossing activates bilateral settling in both directions",
         failures);
  expect(p.vq_sign_changes >= 1 && n.vq_sign_changes >= 1,
         "actuator command reverses across bilateral correction", failures);
  expect(p.max_abs_error_deg < 5.0 && n.max_abs_error_deg < 5.0,
         "200-ms commissioning window remains locally bounded", failures);
  expect(p.max_abs_wheel_rad_s < 20.0 && n.max_abs_wheel_rad_s < 20.0,
         "200-ms commissioning window avoids wheel runaway", failures);

  double mirror_error = 0.0;
  for (std::size_t i = 0; i < positive.size(); ++i) {
    mirror_error = std::max(
        mirror_error,
        std::fabs(positive[i].state.theta_error_rad +
                  negative[i].state.theta_error_rad));
    mirror_error = std::max(
        mirror_error,
        std::fabs(positive[i].state.theta_rate_rad_s +
                  negative[i].state.theta_rate_rad_s));
    mirror_error = std::max(
        mirror_error,
        std::fabs(positive[i].state.wheel_rate_rad_s +
                  negative[i].state.wheel_rate_rad_s));
    mirror_error = std::max(
        mirror_error,
        std::fabs(static_cast<double>(positive[i].output.vq_v) +
                  static_cast<double>(negative[i].output.vq_v)));
  }
  expect(mirror_error < 1.0e-4,
         "closed-loop positive/negative scenarios preserve bilateral symmetry",
         failures);
}

void checkLongRunBalance(int& failures) {
  const PlantModel plant = provisionalNominal();
  const auto evidence = runClosedLoop(
      plant, State{3.0 / kRadToDeg, -0.5, 0.0, 0.0},
      kLongRunGateDurationS);
  const RunMetrics metrics = summarize(evidence);
  const bool pass = longRunBalanceGatePass(evidence, metrics);
  expect(pass, "10-second nominal production-controller balance gate", failures);
  std::cout << std::fixed << std::setprecision(6)
            << "SITL long-run metrics: max_error_deg="
            << metrics.max_abs_error_deg
            << " tail_max_error_deg=" << metrics.tail_max_abs_error_deg
            << " max_rate_rad_s=" << metrics.max_abs_rate_rad_s
            << " max_wheel_rad_s=" << metrics.max_abs_wheel_rad_s
            << " max_vq_v=" << metrics.max_abs_vq_v
            << " zero_crossings=" << metrics.zero_crossings
            << " vq_sign_changes=" << metrics.vq_sign_changes
            << " stable_seen=" << boolText(metrics.stable_seen)
            << " target_sat=" << boolText(metrics.any_target_saturation)
            << " vq_sat=" << boolText(metrics.any_vq_saturation) << '\n';
}

int runSelfTest() {
  int failures = 0;
  checkControllerInvariants(failures);
  checkShortClosedLoop(failures);
  checkLongRunBalance(failures);
  if (failures == 0) {
    std::cout << "PASS deterministic standup SITL\n";
    return 0;
  }
  std::cerr << "FAIL deterministic standup SITL failures=" << failures << '\n';
  return 1;
}

void usage(const char* argv0) {
  std::cout
      << "usage:\n"
      << "  " << argv0 << " --self-test\n"
      << "  " << argv0
      << " --scenario near-upright-positive|near-upright-negative|balance-demo"
         " [--profile nominal|B|C] [--duration-ms N] [--output path]"
         " [--require-balance-gate]\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") {
      return runSelfTest();
    }

    std::string scenario;
    std::string profile = "nominal";
    std::string output_path;
    int duration_ms = 10000;
    bool require_balance_gate = false;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      auto requireValue = [&](const char* option) -> std::string {
        if (i + 1 >= argc) {
          throw std::runtime_error(std::string("missing value for ") + option);
        }
        return argv[++i];
      };
      if (arg == "--scenario") {
        scenario = requireValue("--scenario");
      } else if (arg == "--profile") {
        profile = requireValue("--profile");
      } else if (arg == "--duration-ms") {
        duration_ms = std::stoi(requireValue("--duration-ms"));
      } else if (arg == "--output") {
        output_path = requireValue("--output");
      } else if (arg == "--require-balance-gate") {
        require_balance_gate = true;
      } else if (arg == "--help" || arg == "-h") {
        usage(argv[0]);
        return 0;
      } else {
        throw std::runtime_error("unknown argument: " + arg);
      }
    }

    if (scenario.empty()) {
      usage(argv[0]);
      return 2;
    }
    if (duration_ms <= 0 || duration_ms > 60000) {
      throw std::runtime_error("duration-ms must be in 1..60000");
    }

    State initial{};
    if (scenario == "near-upright-positive") {
      initial = State{1.0 / kRadToDeg, -0.2, 0.0, 0.0};
    } else if (scenario == "near-upright-negative") {
      initial = State{-1.0 / kRadToDeg, 0.2, 0.0, 0.0};
    } else if (scenario == "balance-demo") {
      initial = State{3.0 / kRadToDeg, -0.5, 0.0, 0.0};
    } else {
      throw std::runtime_error("unknown scenario: " + scenario);
    }

    const PlantModel plant = plantByName(profile);
    const auto evidence = runClosedLoop(
        plant, initial, static_cast<double>(duration_ms) * 1.0e-3);
    const RunMetrics metrics = summarize(evidence);
    const bool gate_pass = longRunBalanceGatePass(evidence, metrics);
    if (!output_path.empty()) writeCsv(output_path, evidence);

    std::cout << std::fixed << std::setprecision(6)
              << "scenario=" << scenario << " profile=" << plant.name
              << " samples=" << evidence.size()
              << " max_error_deg=" << metrics.max_abs_error_deg
              << " tail_max_error_deg=" << metrics.tail_max_abs_error_deg
              << " max_rate_rad_s=" << metrics.max_abs_rate_rad_s
              << " max_wheel_rad_s=" << metrics.max_abs_wheel_rad_s
              << " max_vq_v=" << metrics.max_abs_vq_v
              << " zero_crossings=" << metrics.zero_crossings
              << " vq_sign_changes=" << metrics.vq_sign_changes
              << " settling_seen=" << boolText(metrics.settling_seen)
              << " stable_seen=" << boolText(metrics.stable_seen)
              << " all_balance=" << boolText(metrics.all_balance)
              << " balance_gate=" << (gate_pass ? "PASS" : "FAIL") << '\n';
    if (require_balance_gate && !gate_pass) return 1;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "standup SITL error: " << error.what() << '\n';
    return 2;
  }
}
