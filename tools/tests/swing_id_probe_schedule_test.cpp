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

void testVendorScalePumpEnvelope() {
  triwhirl::SwingIdConfig config{};
  assert(config.pump_polarity == 1);
  expectNear(config.pump_v_high, 0.42F);
  expectNear(config.pump_v_low, 0.168F);

  triwhirl::SwingIdRunner runner(config);
  triwhirl::SwingIdInput input{};
  input.now_us = 1000U;
  input.theta_rad = 0.0F;
  input.theta_rate_rad_s = 0.2F;
  input.attitude_valid = true;
  assert(runner.start(input));

  // At theta=0 the nearest configured vertex is about 52 deg away, so the
  // known-good full swing excitation is used and follows the body-rate sign.
  auto output = runner.output();
  expectNear(output.desired_vq_v, config.pump_v_high);

  // Inside the 18 deg near-vertex envelope but outside the 8 deg probe capture
  // window, excitation drops to the vendor-style 0.42/2.5 value.
  input.now_us += 1000U;
  input.theta_rad = degToRad(-36.0F);  // 16 deg from vertex B at -52 deg.
  output = runner.update(input);
  assert(output.state == triwhirl::SwingIdState::kPump);
  expectNear(std::fabs(output.desired_vq_v), config.pump_v_low);
}

void testRateChatterDoesNotCreateHalfCycles() {
  triwhirl::SwingIdConfig config{};
  config.max_duration_us = 2000000U;

  triwhirl::SwingIdRunner runner(config);
  triwhirl::SwingIdInput input{};
  input.now_us = 1000U;
  input.theta_rad = 0.0F;
  input.theta_rate_rad_s = 0.0F;
  input.attitude_valid = true;
  assert(runner.start(input));

  // swing-native-01 showed millisecond-scale rate-sign chatter while the body
  // barely moved. Alternating signs with sub-degree excursion must not be
  // promoted to mechanical half-cycles.
  for (int sample = 0; sample < 40; ++sample) {
    input.now_us += 5000U;
    input.theta_rad = degToRad((sample & 1) ? 0.4F : -0.4F);
    input.theta_rate_rad_s = (sample & 1) ? 0.2F : -0.2F;
    const auto output = runner.update(input);
    assert(output.half_cycle_index == 0U);
  }

  // One opposite rate sign that persists for >20 ms, after >80 ms dwell and
  // with a real angular excursion, is a genuine turn.
  input.now_us += 100000U;
  input.theta_rad = degToRad(5.0F);
  input.theta_rate_rad_s = -0.2F;
  auto output = runner.update(input);
  assert(output.half_cycle_index == 0U);

  input.now_us += 25000U;
  output = runner.update(input);
  assert(output.half_cycle_index == 1U);
}

void testProbeScheduleAndRearm() {
  triwhirl::SwingIdConfig config{};
  config.target_captures = 6U;
  config.probe_duration_us = 1000U;
  config.max_duration_us = 2000000U;
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

    // Rearm still needs only one genuine body turning point, but the turn must
    // now satisfy dwell, persistence, and excursion gates.
    const float turn_rate = (capture % 2U) == 0U ? -0.2F : 0.2F;
    input.now_us += 100000U;
    input.theta_rad = degToRad(vertices_deg[capture] + 20.0F);
    input.theta_rate_rad_s = turn_rate;
    output = runner.update(input);
    assert(output.state == triwhirl::SwingIdState::kRearm);

    input.now_us += 25000U;
    input.theta_rad = degToRad(vertices_deg[capture] + 20.0F);
    input.theta_rate_rad_s = turn_rate;
    output = runner.update(input);
    assert(output.state == triwhirl::SwingIdState::kPump);
    assert(output.pump_active);
    expectNear(std::fabs(output.desired_vq_v), config.pump_v_high);
  }
}

}  // namespace

int main() {
  testVendorScalePumpEnvelope();
  testRateChatterDoesNotCreateHalfCycles();
  testProbeScheduleAndRearm();
  return 0;
}
