// Global swing-up surrogate shared by the batch full-standup runner and the
// continuous WebUI runner.
//
// This file is intentionally included after standup_sitl.cpp so it can reuse the
// native State/PlantModel types and the exact same production StandupController
// translation unit.  It is not a separately fitted plant and must not be treated
// as a digital twin.
//
// The far-field model supplies only the missing global rocking/energy geometry:
//   * 120-degree periodic gravity-like restoring potential;
//   * finite body/wheel dissipation;
//   * bounded Vq-to-body/wheel authority sufficient for the vendor pump law.
//
// Inside +/-2 deg it is exactly the existing identified local B/C plant.  A
// smooth 2..5 deg blend connects the global surrogate to that local evidence.
// The resulting hybrid is therefore a commissioning model for validating the
// sequence Rest -> Swing -> Capture -> Settling -> Stable, not a claim that the
// physical Reuleaux contact mechanics have been identified globally.

namespace {

constexpr double kFullStandupInitialErrorDeg = -59.0;
constexpr double kFullStandupGateDurationS = 12.0;

// Far-field periodic surrogate coefficients.  The gravity-like term is written
// (k/3)*sin(3*theta), giving exactly 120-degree periodicity, unstable upright
// points at 0,+/-120,... and stable resting orientations at +/-60 deg.
constexpr double kGlobalSwingGravityRateS2 = 5.0;
constexpr double kGlobalSwingBodyDampingS1 = 1.5;
constexpr double kGlobalSwingBodyVqGainRadS2PerV = 4.0;
constexpr double kGlobalSwingWheelVqGainRadS2PerV = 30.0;
constexpr double kGlobalSwingWheelDragS1 = 5.0;
constexpr double kGlobalLocalFullDeg = 2.0;
constexpr double kGlobalLocalZeroDeg = 5.0;

inline double wrapGlobalErrorRad(const double value) {
  const double period = 2.0 * kPi / 3.0;
  const double half = kPi / 3.0;
  double wrapped = std::fmod(value + half, period);
  if (wrapped < 0.0) wrapped += period;
  return wrapped - half;
}

inline double smoothstep01(const double x) {
  const double t = std::clamp(x, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

inline double localBlendWeight(const double wrapped_error_rad) {
  const double error_deg = std::fabs(wrapped_error_rad) * kRadToDeg;
  if (error_deg <= kGlobalLocalFullDeg) return 1.0;
  if (error_deg >= kGlobalLocalZeroDeg) return 0.0;
  return smoothstep01((kGlobalLocalZeroDeg - error_deg) /
                      (kGlobalLocalZeroDeg - kGlobalLocalFullDeg));
}

State fullStandupInitialState() {
  return State{kFullStandupInitialErrorDeg / kRadToDeg, 0.0, 0.0, 0.0};
}

State globalStandupDerivative(const PlantModel& local_plant,
                              const State& state, const double vq_v) {
  const double error = wrapGlobalErrorRad(state.theta_error_rad);
  const double rate = state.theta_rate_rad_s;
  const double wheel = state.wheel_rate_rad_s;

  // Far-field rocking surrogate.  Positive Vq follows the measured software
  // actuator coordinate used by the current local plant and controller.
  const double far_body_accel =
      (kGlobalSwingGravityRateS2 / 3.0) * std::sin(3.0 * error) -
      kGlobalSwingBodyDampingS1 * rate +
      kGlobalSwingBodyVqGainRadS2PerV * vq_v;
  const double far_wheel_accel =
      kGlobalSwingWheelVqGainRadS2PerV * vq_v -
      kGlobalSwingWheelDragS1 * wheel;

  // Identified local plant, evaluated in the same periodic upright coordinate.
  const std::array<double, 3> x{{error, rate, wheel}};
  std::array<double, 3> local_dx{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      local_dx[row] += local_plant.a[row][col] * x[col];
    }
    local_dx[row] += local_plant.b[row] * vq_v;
  }

  const double local_weight = localBlendWeight(error);
  const double body_accel =
      local_weight * local_dx[1] + (1.0 - local_weight) * far_body_accel;
  const double wheel_accel =
      local_weight * local_dx[2] + (1.0 - local_weight) * far_wheel_accel;
  return State{rate, body_accel, wheel_accel, wheel};
}

State rk4GlobalStandupStep(const PlantModel& local_plant, const State& state,
                           const double vq_v, const double dt_s) {
  const State k1 = globalStandupDerivative(local_plant, state, vq_v);
  const State k2 = globalStandupDerivative(
      local_plant, addScaled(state, k1, 0.5 * dt_s), vq_v);
  const State k3 = globalStandupDerivative(
      local_plant, addScaled(state, k2, 0.5 * dt_s), vq_v);
  const State k4 = globalStandupDerivative(
      local_plant, addScaled(state, k3, dt_s), vq_v);
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

struct FullStandupMetrics {
  double max_abs_wheel_rad_s = 0.0;
  double max_abs_vq_v = 0.0;
  double tail_max_abs_error_deg = 0.0;
  double first_balance_s = -1.0;
  double first_stable_s = -1.0;
  int body_rate_reversals = 0;
  bool swing_high_seen = false;
  bool swing_low_seen = false;
  bool balance_seen = false;
  bool settling_seen = false;
  bool stable_seen = false;
  bool any_target_saturation = false;
  bool any_vq_saturation = false;
  bool ended_balance = false;
  bool ended_stable = false;
};

FullStandupMetrics summarizeFullStandup(
    const std::vector<EvidenceSample>& evidence) {
  FullStandupMetrics metrics{};
  const double end_t = evidence.empty() ? 0.0 : evidence.back().t_s;
  const double tail_start = std::max(0.0, end_t - 2.0);
  int previous_rate_sign = 0;
  for (const auto& sample : evidence) {
    const double error_deg =
        std::fabs(wrapGlobalErrorRad(sample.state.theta_error_rad)) * kRadToDeg;
    if (sample.t_s >= tail_start) {
      metrics.tail_max_abs_error_deg =
          std::max(metrics.tail_max_abs_error_deg, error_deg);
    }
    metrics.max_abs_wheel_rad_s =
        std::max(metrics.max_abs_wheel_rad_s,
                 std::fabs(sample.state.wheel_rate_rad_s));
    metrics.max_abs_vq_v =
        std::max(metrics.max_abs_vq_v,
                 std::fabs(static_cast<double>(sample.output.vq_v)));
    metrics.swing_high_seen =
        metrics.swing_high_seen ||
        sample.output.phase == triwhirl::StandupPhase::kSwingHigh;
    metrics.swing_low_seen =
        metrics.swing_low_seen ||
        sample.output.phase == triwhirl::StandupPhase::kSwingLow;
    metrics.balance_seen =
        metrics.balance_seen ||
        sample.output.phase == triwhirl::StandupPhase::kBalance;
    metrics.settling_seen = metrics.settling_seen || sample.output.settling;
    metrics.stable_seen = metrics.stable_seen || sample.output.stable;
    metrics.any_target_saturation =
        metrics.any_target_saturation || sample.output.target_saturated;
    metrics.any_vq_saturation =
        metrics.any_vq_saturation || sample.output.vq_saturated;
    if (metrics.first_balance_s < 0.0 &&
        sample.output.phase == triwhirl::StandupPhase::kBalance) {
      metrics.first_balance_s = sample.t_s;
    }
    if (metrics.first_stable_s < 0.0 && sample.output.stable) {
      metrics.first_stable_s = sample.t_s;
    }

    const int rate_sign = signWithDeadband(sample.state.theta_rate_rad_s, 1.0e-4);
    if (rate_sign != 0) {
      if (previous_rate_sign != 0 && rate_sign != previous_rate_sign &&
          sample.output.phase != triwhirl::StandupPhase::kBalance) {
        ++metrics.body_rate_reversals;
      }
      previous_rate_sign = rate_sign;
    }
  }
  if (!evidence.empty()) {
    metrics.ended_balance =
        evidence.back().output.phase == triwhirl::StandupPhase::kBalance;
    metrics.ended_stable = evidence.back().output.stable;
  }
  return metrics;
}

bool fullStandupGatePass(const std::vector<EvidenceSample>& evidence,
                         const FullStandupMetrics& metrics) {
  if (evidence.empty() ||
      evidence.back().t_s + 1.0e-9 < kFullStandupGateDurationS) {
    return false;
  }
  return metrics.swing_high_seen && metrics.swing_low_seen &&
         metrics.body_rate_reversals >= 1 && metrics.balance_seen &&
         metrics.settling_seen && metrics.stable_seen && metrics.ended_balance &&
         metrics.ended_stable && metrics.first_balance_s > 0.5 &&
         metrics.first_stable_s > metrics.first_balance_s &&
         metrics.tail_max_abs_error_deg < 0.50 &&
         metrics.max_abs_wheel_rad_s < 20.0 && metrics.max_abs_vq_v <= 4.0 &&
         !metrics.any_vq_saturation;
}

}  // namespace
