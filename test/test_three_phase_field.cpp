#include "triwhirl/three_phase_field.hpp"

#include <cassert>
#include <cmath>
#include <limits>

namespace {
constexpr float kPi = 3.14159265358979323846F;

bool near(float a, float b, float eps = 1.0e-5F) {
  return std::fabs(a - b) <= eps;
}
}

int main() {
  constexpr float limit = 3.0F;
  constexpr float amplitude = 1.0F;
  constexpr float center = limit * 0.5F;

  for (int i = 0; i < 3600; ++i) {
    const float angle = (2.0F * kPi * static_cast<float>(i)) / 3600.0F;
    const auto p = triwhirl::makeRotatingField(angle, amplitude, limit);
    assert(p.a >= 0.0F && p.a <= limit);
    assert(p.b >= 0.0F && p.b <= limit);
    assert(p.c >= 0.0F && p.c <= limit);
    assert(near(p.a + p.b + p.c, 3.0F * center, 2.0e-5F));
  }

  const auto clamped = triwhirl::makeRotatingField(0.0F, 100.0F, limit);
  assert(clamped.a >= 0.0F && clamped.a <= limit);
  assert(clamped.b >= 0.0F && clamped.b <= limit);
  assert(clamped.c >= 0.0F && clamped.c <= limit);

  const auto invalid = triwhirl::makeRotatingField(
      std::numeric_limits<float>::quiet_NaN(), 1.0F, limit);
  assert(invalid.a == 0.0F && invalid.b == 0.0F && invalid.c == 0.0F);

  return 0;
}
