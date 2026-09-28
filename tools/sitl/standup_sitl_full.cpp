// Deterministic geometry-derived global standup observation runner.
//
// The identified local balance SITL remains the regression authority near
// upright.  This executable starts from the physical Reuleaux resting
// orientation and exercises the production StandupController through a global
// rolling model whose contact/gravity/inertia come from Reuleaux geometry.
//
// Unlike the retired hand-tuned periodic surrogate, a successful full sequence
// is NOT a validation gate.  The far-field model still lacks measured COM offset
// and rolling-loss parameters.  CI gates geometry invariants and local anchors;
// the rest->swing->capture outcome is reported as an observation only.
#define main triwhirl_standup_sitl_embedded_main
#include "standup_sitl.cpp"
#undef main

#include "global_standup_model.inc"

namespace {

std::vector<EvidenceSample> runFullStandupClosedLoop(const PlantModel& local_plant,
                                                     const double duration_s) {
  if (!(duration_s > 0.0)) {
    throw std::runtime_error("duration must be positive");
  }
  const auto config = triwhirl::makeStandupCommissioningConfig(
      static_cast<float>(kThetaReferenceRad), 4.0F);
  triwhirl::StandupController controller(config);
  State state = fullStandupInitialState();
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
      state = rk4GlobalStandupStep(local_plant, state,
                                   static_cast<double>(output.vq_v),
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
      throw std::runtime_error("non-finite full-standup state/output");
    }
  }
  return evidence;
}

void writeFullStandupCsv(const std::string& path,
                         const std::vector<EvidenceSample>& evidence) {
  std::ofstream stream(path);
  if (!stream) throw std::runtime_error("cannot open output: " + path);
  stream << "t_s,true_error_rad,true_error_deg,true_body_angle_rad,"
            "true_body_angle_deg,true_theta_rate_rad_s,true_wheel_rate_rad_s,"
            "true_wheel_angle_rad,center_height_m,contact_body_x_m,"
            "contact_body_y_m,phase,settling,stable,filtered_rate_rad_s,"
            "target_velocity_rad_s,velocity_error_rad_s,velocity_integral_v,"
            "vq_unclamped_v,vq_target_v,vq_applied_v,target_saturated,"
            "vq_saturated\n";
  stream << std::setprecision(10);
  for (const auto& sample : evidence) {
    const double error_rad = wrapGlobalErrorRad(sample.state.theta_error_rad);
    const auto geometry = triwhirl_sitl::reuleauxGroundContact(
        sample.state.theta_error_rad, kReuleauxWidthM);
    const auto& output = sample.output;
    stream << sample.t_s << ',' << error_rad << ',' << error_rad * kRadToDeg
           << ',' << sample.state.theta_error_rad << ','
           << sample.state.theta_error_rad * kRadToDeg << ','
           << sample.state.theta_rate_rad_s << ','
           << sample.state.wheel_rate_rad_s << ','
           << sample.state.wheel_angle_rad << ','
           << geometry.center_height_m << ',' << geometry.body_point_m.x << ','
           << geometry.body_point_m.y << ','
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

int runGeometrySelfTest() {
  const bool nominal = geometryDerivedPlantChecksPass(provisionalNominal());
  const bool b = geometryDerivedPlantChecksPass(provisionalB());
  const bool c = geometryDerivedPlantChecksPass(provisionalC());
  std::cout << (nominal ? "PASS" : "FAIL")
            << " Reuleaux geometry + nominal local anchors\n";
  std::cout << (b ? "PASS" : "FAIL")
            << " Reuleaux geometry + B local anchors\n";
  std::cout << (c ? "PASS" : "FAIL")
            << " Reuleaux geometry + C local anchors\n";
  if (nominal && b && c) {
    std::cout << "PASS geometry-derived global plant invariants\n";
    return 0;
  }
  std::cerr << "FAIL geometry-derived global plant invariants\n";
  return 1;
}

void usageFull(const char* argv0) {
  std::cout << "usage:\n"
            << "  " << argv0 << " --self-test\n"
            << "  " << argv0
            << " [--profile nominal|B|C] [--duration-ms N] [--output path]\n"
            << "note: full swing-up outcome is observational; the retired "
               "--require-standup-gate flag is intentionally unsupported.\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--self-test") {
      return runGeometrySelfTest();
    }

    std::string profile = "nominal";
    std::string output_path;
    int duration_ms =
        static_cast<int>(std::llround(kFullStandupObservationDurationS * 1000.0));
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      auto requireValue = [&](const char* option) -> std::string {
        if (i + 1 >= argc) {
          throw std::runtime_error(std::string("missing value for ") + option);
        }
        return argv[++i];
      };
      if (arg == "--profile") {
        profile = requireValue("--profile");
      } else if (arg == "--duration-ms") {
        duration_ms = std::stoi(requireValue("--duration-ms"));
      } else if (arg == "--output") {
        output_path = requireValue("--output");
      } else if (arg == "--require-standup-gate") {
        throw std::runtime_error(
            "--require-standup-gate was retired: global swing-up is exploratory, "
            "not validation authority");
      } else if (arg == "--help" || arg == "-h") {
        usageFull(argv[0]);
        return 0;
      } else {
        throw std::runtime_error("unknown argument: " + arg);
      }
    }
    if (duration_ms <= 0 || duration_ms > 60000) {
      throw std::runtime_error("duration-ms must be in 1..60000");
    }

    const PlantModel plant = plantByName(profile);
    const bool geometry_ok = geometryDerivedPlantChecksPass(plant);
    if (!geometry_ok) {
      throw std::runtime_error("geometry-derived plant invariant check failed");
    }

    const auto evidence = runFullStandupClosedLoop(
        plant, static_cast<double>(duration_ms) * 1.0e-3);
    const FullStandupMetrics metrics = summarizeFullStandup(evidence);
    const bool sequence_observed = fullStandupSequenceObserved(metrics);
    if (!output_path.empty()) writeFullStandupCsv(output_path, evidence);

    std::cout << std::fixed << std::setprecision(6)
              << "scenario=full-standup profile=" << plant.name
              << " samples=" << evidence.size()
              << " initial_error_deg=" << kFullStandupInitialErrorDeg
              << " first_balance_s=" << metrics.first_balance_s
              << " first_stable_s=" << metrics.first_stable_s
              << " body_rate_reversals=" << metrics.body_rate_reversals
              << " max_wheel_rad_s=" << metrics.max_abs_wheel_rad_s
              << " max_vq_v=" << metrics.max_abs_vq_v
              << " tail_max_error_deg=" << metrics.tail_max_abs_error_deg
              << " swing_high_seen=" << boolText(metrics.swing_high_seen)
              << " swing_low_seen=" << boolText(metrics.swing_low_seen)
              << " balance_seen=" << boolText(metrics.balance_seen)
              << " settling_seen=" << boolText(metrics.settling_seen)
              << " stable_seen=" << boolText(metrics.stable_seen)
              << " ended_balance=" << boolText(metrics.ended_balance)
              << " ended_stable=" << boolText(metrics.ended_stable)
              << " geometry_gate=" << (geometry_ok ? "PASS" : "FAIL")
              << " sequence_observed=" << (sequence_observed ? "YES" : "NO")
              << " validation_authority=NONE\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "full standup SITL error: " << error.what() << '\n';
    return 2;
  }
}
