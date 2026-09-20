#pragma once

#include <cstdint>

#include "driver/mcpwm_prelude.h"

namespace triwhirl {
namespace motor {

class ThreePwmBridge {
 public:
  bool init(int gpio_a, int gpio_b, int gpio_c, std::uint32_t pwm_hz,
            float bus_voltage_v);
  void stopZeroVector();
  bool setPhaseVoltages(float a_v, float b_v, float c_v);

 private:
  bool setDuty(std::size_t phase, float duty);
  bool releaseOutputs();
  mcpwm_timer_handle_t timer_ = nullptr;
  mcpwm_oper_handle_t operators_[3]{};
  mcpwm_cmpr_handle_t comparators_[3]{};
  mcpwm_gen_handle_t generators_[3]{};
  std::uint32_t period_ticks_ = 0U;
  float bus_voltage_v_ = 0.0F;
  bool initialized_ = false;
  bool outputs_forced_low_ = true;
};

}  // namespace motor
}  // namespace triwhirl
