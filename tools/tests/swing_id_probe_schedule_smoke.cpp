#include <cassert>
#include <cmath>
#include <cstdint>

#include "triwhirl/swing_id.hpp"

namespace {

constexpr float kPi = 3.14159265358979323846F;

float degToRad(const float degrees) {
  return degrees * kPi / 180.0F;
}

void expectNear(const float actual, const float expected) {
  assert(std::fabs(actual - expected) < 1.0e-6F);
}

}  // namespace

int main() {
  triwhirl::SwingIdConfig config{};
  config.target_captures = 6U;
  config.probe_duration_us = 1000U;
  config.max_duration_us = 1000000U;
  config.probe_v_negative = -0.25F;
  config.probe_v_positive = 0.25F;

  triwhirl::SwingIdRunner runner(config);
  triwhirl::SwingIdInput input{};
  input.now_us = 1000U;
  input.theta_rad = 0.0F;
  input.theta_rate_rad_s = 0.0F;
  input.attitude_valid = true;
  input.safety_faulted = false;
  assert(runner.start(input));

  const float vertices_deg[6] = {68.0F, -52.0F, 68.0F,
                                  -52.0F, 68.0F, -52.0F};
  const float expected_probe_vq[6] = {-0.25F, -0.25F, 0.25F,
                                       0.25F, 0.0F, 0.0F};

  for (std::uint32_t capture = 0U; capture < 6U; ++capture) {
    input.now_us += 1000U;
    input.theta_rad = degToRad(vertices_deg[capture]);
    auto output = runner.update(input);
    assert(output.state == triwhirl::SwingIdState::kProbe);
    assert(output.probe_active);
    assert(!output.pump_active);
    expectNear(output.desired_vq_v, expected_probe_vq[capture]);

    input.now_us += config.probe_duration_us;
    output = runner.update(input);
    assert(output.capture_count == capture + 1U);

    if (capture + 1U == config.target_captures) {
      assert(output.state == triwhirl::SwingIdState::kComplete);
      expectNear(output.desired_vq_v, 0.0F);
      break;
    }

    assert(output.state == triwhirl::SwingIdState::kRearm);
    assert(output.pump_active);
    assert(!output.probe_active);
    expectNear(std::fabs(output.desired_vq_v), config.pump_v_high);

    // One genuine body turning point at full pump amplitude is enough to
    // re-arm the next probe.  This prevents immediate same-pass probing while
    // preserving high-energy vertex crossings observed on hardware.
    input.now_us += 1000U;
    input.theta_rad = degToRad(vertices_deg[capture] + 20.0F);
    input.theta_rate_rad_s = -0.2F;
    output = runner.update(input);
    assert(output.state == triwhirl::SwingIdState::kPump);
    assert(output.pump_active);
    expectNear(std::fabs(output.desired_vq_v), config.pump_v_high);
  }

  return 0;
}
