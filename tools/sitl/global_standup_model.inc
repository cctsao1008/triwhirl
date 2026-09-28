// Geometry-derived global rolling model shared by the batch full-standup runner
// and the continuous WebUI runner.
//
// This file is intentionally included after standup_sitl.cpp so it can reuse the
// native State/PlantModel types and the exact same production StandupController
// translation unit.
//
// Important evidence boundary:
//   * contact/support geometry is computed from an ideal constant-width Reuleaux
//     triangle, not from a hand-written sin(3*theta) surrogate;
//   * no-slip horizontal translation follows the actual instantaneous contact;
//   * gravity comes from COM height h(theta) and the Lagrange rolling inertia;
//   * the unknown effective body inertia and motor/body authority are anchored to
//     the selected existing B/C local fit at upright;
//   * far-field wheel electrical dynamics retain the selected local fit's direct
//     wheel self-rate and Vq terms;
//   * only a narrow +/-2..5 deg residual blend restores the complete identified
//     local derivative near upright.
//
// This is materially more physical than the retired periodic surrogate, but it
// is still not a validated digital twin: the nominal 75 mm width is seller-level
// geometry, COM offset is assumed zero, rolling loss is not identified, and the
// local B/C fits remain provisional.

#include "reuleaux_rolling_geometry.hpp"

namespace {

constexpr double kFullStandupInitialErrorDeg = -60.0;
constexpr double kFullStandupObservationDurationS = 12.0;
constexpr double kReuleauxWidthM = 0.075;  // seller nominal outer size ~75 mm
constexpr double kGravityMps2 = 9.80665;
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

struct GeometryPlantCalibration {
  double upright_generalized_inertia_per_mass_m2 = 0.0;
  double effective_body_inertia_per_mass_m2 = 0.0;
  double vq_specific_torque_per_v = 0.0;
  double wheel_specific_torque_per_rad_s = 0.0;
};

GeometryPlantCalibration geometryPlantCalibration(const PlantModel& local_plant) {
  const auto upright =
      triwhirl_sitl::reuleauxGroundContact(0.0, kReuleauxWidthM);
  const double gravity_stiffness = local_plant.a[1][0];
  if (!(gravity_stiffness > 0.0) || !std::isfinite(gravity_stiffness)) {
    throw std::runtime_error(
        "geometry global plant requires positive upright gravity stiffness");
  }

  // At theta=0 the active contact is the lower vertex and M'(0)=0.  The
  // geometry-derived linearization is:
  //   theta_ddot ~= (g * h0 / M0) * theta.
  // Solve M0 from the selected local-fit A(1,0), then separate the translational
  // contact-radius contribution to obtain an effective fixed-body inertia per
  // unit mass.  This anchors one otherwise unmeasured inertial parameter to
  // existing evidence instead of inventing it.
  const double m0 =
      kGravityMps2 * upright.center_height_m / gravity_stiffness;
  const double body_inertia_per_mass = m0 - upright.contact_radius_sq_m2;
  if (!(body_inertia_per_mass > 0.0) || !std::isfinite(body_inertia_per_mass)) {
    throw std::runtime_error(
        "geometry global plant inferred non-positive body inertia");
  }

  GeometryPlantCalibration calibration{};
  calibration.upright_generalized_inertia_per_mass_m2 = m0;
  calibration.effective_body_inertia_per_mass_m2 = body_inertia_per_mass;

  // Constant motor torque per V is scaled so its upright body-acceleration gain
  // equals the selected local B(1).  The wheel-rate reaction term is anchored in
  // the same way from local A(1,2).  Both then vary globally only through the
  // geometry-derived rolling inertia M(theta).
  calibration.vq_specific_torque_per_v = local_plant.b[1] * m0;
  calibration.wheel_specific_torque_per_rad_s = local_plant.a[1][2] * m0;
  return calibration;
}

State geometryDerivedFarDerivative(const PlantModel& local_plant,
                                   const State& state, const double vq_v) {
  const GeometryPlantCalibration calibration =
      geometryPlantCalibration(local_plant);
  const auto contact = triwhirl_sitl::reuleauxGroundContact(
      state.theta_error_rad, kReuleauxWidthM);

  const double generalized_inertia_per_mass =
      calibration.effective_body_inertia_per_mass_m2 +
      contact.contact_radius_sq_m2;
  if (!(generalized_inertia_per_mass > 0.0) ||
      !std::isfinite(generalized_inertia_per_mass)) {
    throw std::runtime_error("invalid geometry-derived rolling inertia");
  }

  // Lagrange equation for one no-slip rolling body coordinate:
  //   M(theta) theta_ddot + 0.5 M'(theta) theta_dot^2
  //       + g h'(theta) = Q_specific.
  // h'(theta) comes directly from exact Reuleaux support geometry.  No global
  // gravity sine wave and no arbitrary far-field body damping are inserted.
  const double specific_motor_torque =
      calibration.vq_specific_torque_per_v * vq_v +
      calibration.wheel_specific_torque_per_rad_s * state.wheel_rate_rad_s;
  const double body_accel =
      (specific_motor_torque -
       0.5 * contact.contact_radius_sq_derivative_m2_per_rad *
           state.theta_rate_rad_s * state.theta_rate_rad_s -
       kGravityMps2 * contact.support_derivative_m_per_rad) /
      generalized_inertia_per_mass;

  // Far from upright we retain only the orientation-independent direct wheel
  // terms from the selected local evidence.  Body-angle/body-rate cross terms
  // are local-fit effects and are restored only by the near-upright residual
  // blend below rather than extrapolated around the whole 120-deg cycle.
  const double wheel_accel =
      local_plant.a[2][2] * state.wheel_rate_rad_s +
      local_plant.b[2] * vq_v;

  return State{state.theta_rate_rad_s, body_accel, wheel_accel,
               state.wheel_rate_rad_s};
}

State globalStandupDerivative(const PlantModel& local_plant,
                              const State& state, const double vq_v) {
  const State far = geometryDerivedFarDerivative(local_plant, state, vq_v);
  const double error = wrapGlobalErrorRad(state.theta_error_rad);

  // Exact identified local derivative in the periodic upright coordinate.
  const std::array<double, 3> x{{error, state.theta_rate_rad_s,
                                 state.wheel_rate_rad_s}};
  std::array<double, 3> local_dx{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t col = 0; col < 3; ++col) {
      local_dx[row] += local_plant.a[row][col] * x[col];
    }
    local_dx[row] += local_plant.b[row] * vq_v;
  }

  const double local_weight = localBlendWeight(error);
  const double body_accel =
      local_weight * local_dx[1] + (1.0 - local_weight) * far.theta_rate_rad_s;
  const double wheel_accel =
      local_weight * local_dx[2] + (1.0 - local_weight) * far.wheel_rate_rad_s;
  return State{state.theta_rate_rad_s, body_accel, wheel_accel,
               state.wheel_rate_rad_s};
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

double integrateRollingCenterX(const State& previous, const State& next,
                               const double previous_center_x_m,
                               const double dt_s) {
  const double vx0 = triwhirl_sitl::rollingCenterVelocityX(
      previous.theta_error_rad, previous.theta_rate_rad_s, kReuleauxWidthM);
  const double vx1 = triwhirl_sitl::rollingCenterVelocityX(
      next.theta_error_rad, next.theta_rate_rad_s, kReuleauxWidthM);
  return previous_center_x_m + 0.5 * (vx0 + vx1) * dt_s;
}

State fullStandupInitialState() {
  return State{kFullStandupInitialErrorDeg / kRadToDeg, 0.0, 0.0, 0.0};
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

bool fullStandupSequenceObserved(const FullStandupMetrics& metrics) {
  return metrics.swing_high_seen && metrics.swing_low_seen &&
         metrics.body_rate_reversals >= 1 && metrics.balance_seen &&
         metrics.settling_seen && metrics.stable_seen && metrics.ended_balance &&
         metrics.ended_stable;
}

bool geometryDerivedPlantChecksPass(const PlantModel& plant) {
  double max_width_error = 0.0;
  double max_ground_error = 0.0;
  for (int degree = -180; degree <= 180; ++degree) {
    const double theta = static_cast<double>(degree) * kPi / 180.0;
    const auto down =
        triwhirl_sitl::reuleauxGroundContact(theta, kReuleauxWidthM);
    const auto up = triwhirl_sitl::reuleauxGroundContact(theta + kPi,
                                                         kReuleauxWidthM);
    max_width_error =
        std::max(max_width_error,
                 std::fabs(down.center_height_m + up.center_height_m -
                           kReuleauxWidthM));
    const auto pose =
        triwhirl_sitl::reuleauxRollingPose(theta, 0.0, kReuleauxWidthM);
    max_ground_error = std::max(max_ground_error, std::fabs(pose.contact_y_m));
  }

  const auto upright =
      triwhirl_sitl::reuleauxGroundContact(0.0, kReuleauxWidthM);
  const auto rest = triwhirl_sitl::reuleauxGroundContact(
      kPi / 3.0, kReuleauxWidthM);
  const bool potential_geometry_ok =
      upright.center_height_m > rest.center_height_m + 0.005;

  double max_support_derivative_error = 0.0;
  constexpr std::array<double, 4> kDerivativeAnglesDeg{{10.0, 35.0, 50.0, 58.0}};
  constexpr double kDerivativeStep = 1.0e-6;
  for (const double degree : kDerivativeAnglesDeg) {
    const double theta = degree * kPi / 180.0;
    const auto geometry =
        triwhirl_sitl::reuleauxGroundContact(theta, kReuleauxWidthM);
    const double hp = triwhirl_sitl::reuleauxGroundContact(
                          theta + kDerivativeStep, kReuleauxWidthM)
                          .center_height_m;
    const double hm = triwhirl_sitl::reuleauxGroundContact(
                          theta - kDerivativeStep, kReuleauxWidthM)
                          .center_height_m;
    const double numeric = (hp - hm) / (2.0 * kDerivativeStep);
    max_support_derivative_error =
        std::max(max_support_derivative_error,
                 std::fabs(numeric - geometry.support_derivative_m_per_rad));
  }

  const GeometryPlantCalibration calibration = geometryPlantCalibration(plant);
  const State zero{};
  const State vq_response = geometryDerivedFarDerivative(plant, zero, 1.0);
  const State wheel_response = geometryDerivedFarDerivative(
      plant, State{0.0, 0.0, 1.0, 0.0}, 0.0);

  constexpr double kLinearizationProbeRad = 1.0e-6;
  const State gravity_probe = geometryDerivedFarDerivative(
      plant, State{kLinearizationProbeRad, 0.0, 0.0, 0.0}, 0.0);
  const double gravity_slope =
      gravity_probe.theta_rate_rad_s / kLinearizationProbeRad;

  const bool anchors_ok =
      std::fabs(gravity_slope - plant.a[1][0]) < 0.05 &&
      std::fabs(vq_response.theta_rate_rad_s - plant.b[1]) < 1.0e-9 &&
      std::fabs(wheel_response.theta_rate_rad_s - plant.a[1][2]) < 1.0e-9 &&
      calibration.effective_body_inertia_per_mass_m2 > 0.0;

  return max_width_error < 1.0e-9 && max_ground_error < 1.0e-12 &&
         potential_geometry_ok && max_support_derivative_error < 1.0e-7 &&
         anchors_ok;
}

}  // namespace
