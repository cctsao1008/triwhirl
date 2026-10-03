#include <array>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <string>

#include "local_linear_plant.hpp"

namespace {

struct AccelerationLocalForm {
  bool valid = false;
  double a_theta = 0.0;
  double b_theta_rate = 0.0;
  double c_wheel_rate = 0.0;
  double d_wheel_accel = 0.0;
};

AccelerationLocalForm accelerationLocalForm(
    const triwhirl::sitl::LocalLinearPlant& plant) {
  AccelerationLocalForm form{};
  if (!triwhirl::sitl::validLocalLinearPlant(plant)) return form;

  const double d = plant.b[1] / plant.b[2];
  if (!std::isfinite(d)) return form;

  form.a_theta = plant.a[1][0] - d * plant.a[2][0];
  form.b_theta_rate = plant.a[1][1] - d * plant.a[2][1];
  form.c_wheel_rate = plant.a[1][2] - d * plant.a[2][2];
  form.d_wheel_accel = d;
  form.valid = std::isfinite(form.a_theta) &&
               std::isfinite(form.b_theta_rate) &&
               std::isfinite(form.c_wheel_rate) &&
               std::isfinite(form.d_wheel_accel);
  if (!form.valid) return AccelerationLocalForm{};
  return form;
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

// Necessary Hurwitz coefficient inequalities for
//
//   theta_dot  = theta_rate
//   theta_ddot = a*theta + b*theta_rate + c*wheel_rate + d*u
//   wheel_dot  = u
//   u          = f1*theta + f2*theta_rate + f3*wheel_rate
//
// The characteristic polynomial is
//   lambda^3 + alpha2 lambda^2 + alpha1 lambda + alpha0
// where
//   alpha2 = -(b + d*f2 + f3)
//   alpha1 = -a + b*f3 - c*f2 - d*f1
//   alpha0 =  a*f3 - c*f1
//
// Hurwitz stability requires alpha2, alpha1 and alpha0 all positive. Written
// as strict linear inequalities A*f < rhs, the row order below is:
//   alpha2 > 0, alpha0 > 0, alpha1 > 0.
struct NecessaryInequalities {
  std::array<std::array<double, 3>, 3> a{};
  std::array<double, 3> rhs{};
};

NecessaryInequalities necessaryInequalities(
    const AccelerationLocalForm& form) {
  NecessaryInequalities out{};
  out.a[0] = {{0.0, form.d_wheel_accel, 1.0}};
  out.rhs[0] = -form.b_theta_rate;

  out.a[1] = {{form.c_wheel_rate, 0.0, -form.a_theta}};
  out.rhs[1] = 0.0;

  out.a[2] = {{form.d_wheel_accel, form.c_wheel_rate,
               -form.b_theta_rate}};
  out.rhs[2] = -form.a_theta;
  return out;
}

}  // namespace

int main() {
  int failures = 0;

  const auto plant_b = triwhirl::sitl::provisionalLocalB();
  const auto plant_c = triwhirl::sitl::provisionalLocalC();
  const auto form_b = accelerationLocalForm(plant_b);
  const auto form_c = accelerationLocalForm(plant_c);

  expect(form_b.valid && form_c.valid,
         "provisional B/C transform to wheel-acceleration local form",
         failures);

  if (!form_b.valid || !form_c.valid) return 1;

  std::cout << std::setprecision(15)
            << "B: a=" << form_b.a_theta
            << " b=" << form_b.b_theta_rate
            << " c=" << form_b.c_wheel_rate
            << " d=" << form_b.d_wheel_accel << '\n'
            << "C: a=" << form_c.a_theta
            << " b=" << form_c.b_theta_rate
            << " c=" << form_c.c_wheel_rate
            << " d=" << form_c.d_wheel_accel << '\n';

  const auto inequalities_b = necessaryInequalities(form_b);
  const auto inequalities_c = necessaryInequalities(form_c);

  std::array<std::array<double, 3>, 6> a{};
  std::array<double, 6> rhs{};
  for (std::size_t row = 0; row < 3; ++row) {
    a[row] = inequalities_b.a[row];
    rhs[row] = inequalities_b.rhs[row];
    a[row + 3] = inequalities_c.a[row];
    rhs[row + 3] = inequalities_c.rhs[row];
  }

  // Non-negative Farkas certificate for the current repository coefficients.
  // Non-zero rows are B.alpha2, B.alpha1, C.alpha2 and C.alpha0. The values
  // are a certificate, not controller gains. The test below recomputes both
  // y^T A and y^T rhs from the live plant fixtures, so coefficient changes must
  // re-earn (or invalidate) this model-feasibility conclusion.
  constexpr std::array<double, 6> y{{
      1.0,
      0.0,
      0.179405739172411,
      0.678867933361362,
      0.004912031758055,
      0.0,
  }};

  bool non_negative = true;
  for (const double value : y) {
    non_negative = non_negative && std::isfinite(value) && value >= 0.0;
  }
  expect(non_negative, "Farkas multipliers are finite and non-negative",
         failures);

  std::array<double, 3> weighted_lhs{};
  double weighted_rhs = 0.0;
  for (std::size_t row = 0; row < a.size(); ++row) {
    for (std::size_t col = 0; col < weighted_lhs.size(); ++col) {
      weighted_lhs[col] += y[row] * a[row][col];
    }
    weighted_rhs += y[row] * rhs[row];
  }

  constexpr double kResidualTolerance = 1.0e-9;
  bool zero_lhs = true;
  for (const double value : weighted_lhs) {
    zero_lhs = zero_lhs && std::isfinite(value) &&
               std::fabs(value) <= kResidualTolerance;
  }
  expect(zero_lhs, "certificate cancels all feedback-gain coefficients",
         failures);
  expect(std::isfinite(weighted_rhs) && weighted_rhs < -1.0,
         "certificate produces a strictly negative weighted RHS", failures);

  std::cout << "certificate_residual=[" << weighted_lhs[0] << ','
            << weighted_lhs[1] << ',' << weighted_lhs[2] << "]"
            << " weighted_rhs=" << weighted_rhs << '\n';

  // If A*f < rhs were simultaneously feasible, multiplying every inequality
  // by y >= 0 and summing would give:
  //
  //     (y^T A) f < y^T rhs
  //             0 < negative_number
  //
  // which is impossible. Since these rows encode only necessary coefficient
  // signs (not even the full cubic Hurwitz test), the current B/C fixtures
  // cannot share one static local wheel-acceleration feedback Jacobian.
  const bool infeasibility_certificate =
      non_negative && zero_lhs && weighted_rhs < -1.0;
  expect(infeasibility_certificate,
         "current B/C necessary Hurwitz inequalities are jointly infeasible",
         failures);

  // The first-order target-velocity servo has, locally and away from clamps,
  //   u = (target - wheel_rate) / tau.
  // For any tau > 0 this is a bijection between a target-velocity Jacobian K
  // and acceleration feedback F:
  //   F = {K0/tau, K1/tau, (K2-1)/tau}
  //   K = {tau*F0, tau*F1, 1+tau*F2}.
  // Verify the algebraic round trip without assigning hardware authority to tau.
  constexpr double tau = 0.1;
  constexpr std::array<double, 3> f_probe{{12.0, -3.0, -8.0}};
  const std::array<double, 3> k_probe{{
      tau * f_probe[0],
      tau * f_probe[1],
      1.0 + tau * f_probe[2],
  }};
  const std::array<double, 3> f_round_trip{{
      k_probe[0] / tau,
      k_probe[1] / tau,
      (k_probe[2] - 1.0) / tau,
  }};
  bool servo_bijection = true;
  for (std::size_t i = 0; i < f_probe.size(); ++i) {
    servo_bijection =
        servo_bijection && std::fabs(f_round_trip[i] - f_probe[i]) < 1.0e-12;
  }
  expect(servo_bijection,
         "first-order target-velocity Jacobian maps bijectively to acceleration feedback",
         failures);

  if (failures == 0) {
    std::cout
        << "PASS model-feasibility blocker: current provisional B/C pair has no "
           "shared differentiable memoryless local Jacobian under the first-order "
           "target-velocity servo abstraction\n";
    return 0;
  }

  std::cerr << "FAIL local shared-feedback feasibility contract failures="
            << failures << '\n';
  return 1;
}
