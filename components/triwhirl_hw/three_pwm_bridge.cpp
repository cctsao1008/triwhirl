#include "triwhirl/motor/three_pwm_bridge.hpp"

#include <cmath>

#include "esp_err.h"

namespace triwhirl {
namespace motor {
namespace {
constexpr std::uint32_t kTimerResolutionHz = 10000000U;
float clamp01(const float value) {
  return value < 0.0F ? 0.0F : (value > 1.0F ? 1.0F : value);
}
}  // namespace

bool ThreePwmBridge::init(const int gpio_a, const int gpio_b, const int gpio_c,
                          const std::uint32_t pwm_hz,
                          const float bus_voltage_v) {
  if (pwm_hz == 0U || !(bus_voltage_v > 0.0F) ||
      !std::isfinite(bus_voltage_v)) {
    return false;
  }
  period_ticks_ = kTimerResolutionHz / pwm_hz;
  if (period_ticks_ < 2U) {
    return false;
  }
  bus_voltage_v_ = bus_voltage_v;

  mcpwm_timer_config_t timer_config{};
  timer_config.group_id = 0;
  timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_config.resolution_hz = kTimerResolutionHz;
  timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
  timer_config.period_ticks = period_ticks_;
  if (mcpwm_new_timer(&timer_config, &timer_) != ESP_OK) {
    return false;
  }

  const int gpios[3] = {gpio_a, gpio_b, gpio_c};
  for (std::size_t i = 0; i < 3U; ++i) {
    mcpwm_operator_config_t operator_config{};
    operator_config.group_id = 0;
    if (mcpwm_new_operator(&operator_config, &operators_[i]) != ESP_OK ||
        mcpwm_operator_connect_timer(operators_[i], timer_) != ESP_OK) {
      return false;
    }

    mcpwm_comparator_config_t comparator_config{};
    comparator_config.flags.update_cmp_on_tez = true;
    if (mcpwm_new_comparator(operators_[i], &comparator_config,
                             &comparators_[i]) != ESP_OK) {
      return false;
    }

    mcpwm_generator_config_t generator_config{};
    generator_config.gen_gpio_num = gpios[i];
    if (mcpwm_new_generator(operators_[i], &generator_config,
                            &generators_[i]) != ESP_OK) {
      return false;
    }

    if (mcpwm_generator_set_action_on_timer_event(
            generators_[i], MCPWM_GEN_TIMER_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_UP,
                                MCPWM_TIMER_EVENT_EMPTY,
                                MCPWM_GEN_ACTION_HIGH)) != ESP_OK ||
        mcpwm_generator_set_action_on_compare_event(
            generators_[i], MCPWM_GEN_COMPARE_EVENT_ACTION(
                                MCPWM_TIMER_DIRECTION_UP, comparators_[i],
                                MCPWM_GEN_ACTION_LOW)) != ESP_OK ||
        mcpwm_comparator_set_compare_value(comparators_[i], 0U) != ESP_OK ||
        mcpwm_generator_set_force_level(generators_[i], 0, true) != ESP_OK) {
      return false;
    }
  }

  if (mcpwm_timer_enable(timer_) != ESP_OK ||
      mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP) != ESP_OK) {
    return false;
  }
  outputs_forced_low_ = true;
  initialized_ = true;
  return true;
}

void ThreePwmBridge::stopZeroVector() {
  if (!initialized_ || outputs_forced_low_) {
    return;
  }

  bool forced = true;
  for (mcpwm_gen_handle_t generator : generators_) {
    if (mcpwm_generator_set_force_level(generator, 0, true) != ESP_OK) {
      forced = false;
    }
  }
  outputs_forced_low_ = forced;
}

bool ThreePwmBridge::setDuty(const std::size_t phase, const float duty) {
  if (!initialized_ || phase >= 3U || !std::isfinite(duty)) {
    return false;
  }

  const float clamped = clamp01(duty);
  const std::uint32_t compare = static_cast<std::uint32_t>(
      clamped * static_cast<float>(period_ticks_));
  return mcpwm_comparator_set_compare_value(comparators_[phase], compare) ==
         ESP_OK;
}

bool ThreePwmBridge::releaseOutputs() {
  if (!initialized_) {
    return false;
  }
  if (!outputs_forced_low_) {
    return true;
  }

  for (mcpwm_gen_handle_t generator : generators_) {
    if (mcpwm_generator_set_force_level(generator, -1, true) != ESP_OK) {
      // A partially released bridge is not a valid actuator state. Reassert a
      // deterministic zero vector on every phase before reporting failure.
      for (mcpwm_gen_handle_t rollback : generators_) {
        (void)mcpwm_generator_set_force_level(rollback, 0, true);
      }
      outputs_forced_low_ = true;
      return false;
    }
  }

  outputs_forced_low_ = false;
  return true;
}

bool ThreePwmBridge::setPhaseVoltages(const float a_v, const float b_v,
                                      const float c_v) {
  if (!initialized_ || !std::isfinite(a_v) || !std::isfinite(b_v) ||
      !std::isfinite(c_v)) {
    stopZeroVector();
    return false;
  }

  // While stopped the generators remain forced low. Program a complete new
  // three-phase compare set first, then release the force state once. During
  // active operation each control tick only updates comparator values; it does
  // not touch generator force state. This mirrors the normal ESP32 SimpleFOC
  // MCPWM fast path and removes six unnecessary force-state operations from the
  // 1 kHz control deadline.
  if (!setDuty(0U, a_v / bus_voltage_v_) ||
      !setDuty(1U, b_v / bus_voltage_v_) ||
      !setDuty(2U, c_v / bus_voltage_v_) || !releaseOutputs()) {
    stopZeroVector();
    return false;
  }
  return true;
}

}  // namespace motor
}  // namespace triwhirl
