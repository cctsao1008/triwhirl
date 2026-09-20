#include "triwhirl/upright_geometry.hpp"

#include <cassert>
#include <cmath>

namespace {

bool near(const float a, const float b, const float tol = 1.0e-4F) {
  return std::fabs(a - b) <= tol;
}

}  // namespace

int main() {
  using triwhirl::periodicUprightErrorDeg;
  using triwhirl::periodicUprightErrorRad;

  constexpr float reference_deg = 68.0F;

  // All three physical upright vertices map to the same local equilibrium.
  assert(near(periodicUprightErrorDeg(68.0F, reference_deg), 0.0F));
  assert(near(periodicUprightErrorDeg(-52.0F, reference_deg), 0.0F));
  assert(near(periodicUprightErrorDeg(-172.0F, reference_deg), 0.0F));
  assert(near(periodicUprightErrorDeg(188.0F, reference_deg), 0.0F));

  // Local perturbations preserve sign and magnitude at every vertex.
  assert(near(periodicUprightErrorDeg(73.0F, reference_deg), 5.0F));
  assert(near(periodicUprightErrorDeg(-47.0F, reference_deg), 5.0F));
  assert(near(periodicUprightErrorDeg(-167.0F, reference_deg), 5.0F));
  assert(near(periodicUprightErrorDeg(63.0F, reference_deg), -5.0F));
  assert(near(periodicUprightErrorDeg(-57.0F, reference_deg), -5.0F));
  assert(near(periodicUprightErrorDeg(-177.0F, reference_deg), -5.0F));

  // Periodic boundary contract is [-60, 60).
  assert(near(periodicUprightErrorDeg(reference_deg + 60.0F, reference_deg),
              -60.0F));
  assert(near(periodicUprightErrorDeg(reference_deg - 60.0F, reference_deg),
              -60.0F));

  constexpr float reference_rad = reference_deg * triwhirl::kPi / 180.0F;
  assert(near(periodicUprightErrorRad(reference_rad, reference_rad), 0.0F));
  assert(near(periodicUprightErrorRad(reference_rad + triwhirl::kUprightPeriodRad,
                                      reference_rad),
              0.0F));
  assert(near(periodicUprightErrorRad(reference_rad - triwhirl::kUprightPeriodRad,
                                      reference_rad),
              0.0F));

  return 0;
}
