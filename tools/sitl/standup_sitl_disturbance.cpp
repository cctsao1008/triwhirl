// Deterministic disturbance runner for the TriWhirl standup SITL.
//
// Keep one copy of the provisional plant, controller timing, evidence schema,
// and balance-gate metrics by compiling the existing SITL implementation into
// this translation unit with its CLI entry point renamed.  The only additional
// behavior here is application of discrete body-rate / wheel-rate impulses.
// The production StandupController remains the controller under test.
#define main triwhirl_standup_sitl_embedded_main
#include "standup_sitl.cpp"
#undef main

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

enum class DisturbanceKind {
  kBodyRate,
  kWheelRate,
};

struct DisturbanceEvent {
  double t_s = 0.0;
  double delta_rad_s = 0.0;
  DisturbanceKind kind = DisturbanceKind::kBodyRate;
};

DisturbanceEvent parseDisturbanceSpec(const std::string& text,
                                      const DisturbanceKind kind) {
  const std::size_t separator = text.find(':');
  if (separator == std::string::npos || separator == 0U ||
      separator + 1U >= text.size()) {
    throw std::runtime_error(
        "disturbance must use <time_s>:<delta_rad_s>, got: " + text);
  }
  DisturbanceEvent event{};
  event.t_s = std::stod(text.substr(0U, separator));
  event.delta_rad_s = std::stod(text.substr(separator + 1U));
  event.kind = kind;
  if (!std::isfinite(event.t_s) || !std::isfinite(event.delta_rad_s)) {
    throw std::runtime_error("disturbance contains a non-finite value: " + text);
  }
  if (event.t_s < 0.0) {
    throw std::runtime_error("disturbance time must be non-negative: " + text);
  }
  const double limit =
      kind == DisturbanceKind::kBodyRate ? 5.0 : 100.0;
  if (std::fabs(event.delta_rad_s) > limit) {
    throw std::runtime_error("disturbance magnitude exceeds safety envelope: " +
                             text);
  }
  return event;
}

void applyDisturbance(State& state, const DisturbanceEvent& event) {
  if (event.kind == DisturbanceKind::kBodyRate) {
    state.theta_rate_rad_s += event.delta_rad_s;
  } else {
    state.wheel_rate_rad_s += event.delta_rad_s;
  }
}

std::vector<EvidenceSample> runDisturbedClosedLoop(
    const PlantModel& plant, const State& initial, const double duration_s,
    std::vector<DisturbanceEvent> events) {
  if (!(duration_s > 0.0)) {
    throw std::runtime_error("duration must be positive");
  }
  std::stable_sort(events.begin(), events.end(),
                   [](const DisturbanceEvent& lhs,
                      const DisturbanceEvent& rhs) {
                     return lhs.t_s < rhs.t_s;
                   });
  if (!events.empty() && events.back().t_s > duration_s + 1.0e-12) {
    throw std::runtime_error("disturbance lies beyond simulation duration");
  }

  const auto config = triwhirl::makeStandupCommissioningConfig(
      static_cast<float>(kThetaReferenceRad), 4.0F);
  triwhirl::StandupController controller(config);
  State state = initial;
  std::size_t next_event = 0U;

  // Apply t=0 impulses before controller initialization so the production
  // controller sees the actual disturbed initial state.
  while (next_event < events.size() && events[next_event].t_s <= 1.0e-12) {
    applyDisturbance(state, events[next_event]);
    ++next_event;
  }

  std::uint32_t now_us = kStartTimeUs;
  auto input = controllerInput(now_us, state);
  controller.reset(input);
  auto output = controller.update(input);

  const int control_steps =
      static_cast<int>(std::llround(duration_s / kControlPeriodS));
  std::vector<EvidenceSample> evidence;
  evidence.reserve(static_cast<std::size_t>(control_steps) + 1U);
  evidence.push_back(EvidenceSample{0.0, state, output});

  for (int step = 1; step <= control_steps; ++step) {
    for (int substep = 0; substep < kPlantSubsteps; ++substep) {
      state = rk4Step(plant, state, static_cast<double>(output.vq_v),
                      kPlantStepS);
    }

    const double t_s = static_cast<double>(step) * kControlPeriodS;
    while (next_event < events.size() &&
           events[next_event].t_s <= t_s + 5.0e-7) {
      applyDisturbance(state, events[next_event]);
      ++next_event;
    }

    now_us += kControlPeriodUs;
    input = controllerInput(now_us, state);
    output = controller.update(input);
    if (!output.valid || !std::isfinite(output.vq_v) ||
        !std::isfinite(state.theta_error_rad) ||
        !std::isfinite(state.theta_rate_rad_s) ||
        !std::isfinite(state.wheel_rate_rad_s) ||
        !std::isfinite(state.wheel_angle_rad)) {
      throw std::runtime_error("non-finite disturbed closed-loop state/output");
    }
    evidence.push_back(EvidenceSample{t_s, state, output});
  }
  return evidence;
}

bool disturbanceGatePass(const std::vector<EvidenceSample>& evidence,
                         const RunMetrics& metrics,
                         const std::vector<DisturbanceEvent>& events,
                         const double duration_s) {
  if (events.empty()) return longRunBalanceGatePass(evidence, metrics);
  if (evidence.empty() || evidence.back().t_s + 1.0e-9 < duration_s) {
    return false;
  }
  const double last_event_s = events.back().t_s;
  if (duration_s - last_event_s < 1.0) {
    return false;  // Require visible recovery time after the final kick.
  }

  // Disturbance runs are allowed to leave the tight +/-5-degree commissioning
  // envelope temporarily.  They must never leave Balance, must recover into the
  // existing tail gate, must finish Stable, and must retain bounded momentum.
  return metrics.all_balance && metrics.settling_seen && metrics.stable_seen &&
         evidence.back().output.stable &&
         metrics.max_abs_error_deg < 30.0 &&
         metrics.tail_max_abs_error_deg < 0.50 &&
         metrics.max_abs_wheel_rad_s < 80.0 &&
         metrics.max_abs_vq_v <= 4.0;
}

void usageDisturbance(const char* argv0) {
  std::cout
      << "usage:\n"
      << "  " << argv0
      << " --scenario balance-demo [--profile nominal|B|C]"
         " [--duration-ms N] [--output path]"
         " [--body-kick time_s:delta_rad_s]..."
         " [--wheel-kick time_s:delta_rad_s]..."
         " [--require-disturbance-gate]\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::string scenario = "balance-demo";
    std::string profile = "nominal";
    std::string output_path;
    int duration_ms = 10000;
    bool require_disturbance_gate = false;
    std::vector<DisturbanceEvent> events;

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
      } else if (arg == "--body-kick") {
        events.push_back(parseDisturbanceSpec(
            requireValue("--body-kick"), DisturbanceKind::kBodyRate));
      } else if (arg == "--wheel-kick") {
        events.push_back(parseDisturbanceSpec(
            requireValue("--wheel-kick"), DisturbanceKind::kWheelRate));
      } else if (arg == "--require-disturbance-gate") {
        require_disturbance_gate = true;
      } else if (arg == "--help" || arg == "-h") {
        usageDisturbance(argv[0]);
        return 0;
      } else {
        throw std::runtime_error("unknown argument: " + arg);
      }
    }

    if (scenario != "balance-demo") {
      throw std::runtime_error(
          "disturbance runner currently supports only balance-demo");
    }
    if (duration_ms <= 0 || duration_ms > 60000) {
      throw std::runtime_error("duration-ms must be in 1..60000");
    }

    const double duration_s = static_cast<double>(duration_ms) * 1.0e-3;
    std::stable_sort(events.begin(), events.end(),
                     [](const DisturbanceEvent& lhs,
                        const DisturbanceEvent& rhs) {
                       return lhs.t_s < rhs.t_s;
                     });
    const PlantModel plant = plantByName(profile);
    const State initial{3.0 / kRadToDeg, -0.5, 0.0, 0.0};
    const auto evidence =
        runDisturbedClosedLoop(plant, initial, duration_s, events);
    const RunMetrics metrics = summarize(evidence);
    const bool gate_pass =
        disturbanceGatePass(evidence, metrics, events, duration_s);
    if (!output_path.empty()) writeCsv(output_path, evidence);

    const double last_disturbance_s =
        events.empty() ? -1.0 : events.back().t_s;
    std::cout << std::fixed << std::setprecision(6)
              << "scenario=" << scenario << " profile=" << plant.name
              << " samples=" << evidence.size()
              << " disturbances=" << events.size()
              << " last_disturbance_s=" << last_disturbance_s
              << " max_error_deg=" << metrics.max_abs_error_deg
              << " tail_max_error_deg=" << metrics.tail_max_abs_error_deg
              << " max_rate_rad_s=" << metrics.max_abs_rate_rad_s
              << " max_wheel_rad_s=" << metrics.max_abs_wheel_rad_s
              << " max_vq_v=" << metrics.max_abs_vq_v
              << " settling_seen=" << boolText(metrics.settling_seen)
              << " stable_seen=" << boolText(metrics.stable_seen)
              << " all_balance=" << boolText(metrics.all_balance)
              << " target_sat=" << boolText(metrics.any_target_saturation)
              << " vq_sat=" << boolText(metrics.any_vq_saturation)
              << " disturbance_gate=" << (gate_pass ? "PASS" : "FAIL")
              << '\n';

    if (require_disturbance_gate && !gate_pass) return 1;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "standup disturbance SITL error: " << error.what() << '\n';
    return 2;
  }
}
