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

    // Rearm must restore energy for four genuine body half-cycles before the
    // next probe can be armed.  Keep the body outside the rearm angle while
    // alternating a rate sign above the switch deadband.
    for (std::uint32_t half_cycle = 0U; half_cycle < 4U; ++half_cycle) {
      input.now_us += 1000U;
      input.theta_rad = degToRad(vertices_deg[capture] + 20.0F);
      input.theta_rate_rad_s = (half_cycle % 2U) == 0U ? -0.2F : 0.2F;
      output = runner.update(input);
      expectNear(std::fabs(output.desired_vq_v), config.pump_v_high);
      if (half_cycle < 3U) {
        assert(output.state == triwhirl::SwingIdState::kRearm);
      } else {
        assert(output.state == triwhirl::SwingIdState::kPump);
      }
      assert(output.pump_active);
    }
  }

  return 0;
}
