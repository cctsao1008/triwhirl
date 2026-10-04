// Issue #53/#75/#80 production-shaped Route-B compile/link proof only.
//
// Native ESP-IDF builds compile this translation unit as empty. The Route-B
// environment enables the production-shaped triwhirl_simplefoc component and
// retains the function below at link time. The function is NEVER called by
// app_main(), so it performs no I2C access, FOC initialization, PWM enable, or
// task creation on a running device.
//
// #75 proves the complete inactive target architecture; #80 additionally keeps
// the passive AS5600 commissioning object in the same pinned SimpleFOC graph.

#if defined(TRIWHIRL_ROUTE_B_SIMPLEFOC_LINK_PROBE) && \
    defined(TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND)

#include <cmath>
#include <cstdint>

#include "runtime_motor_task.hpp"
#include "triwhirl/fuzzy_attitude_command.hpp"
#include "triwhirl/fuzzy_balance_seed.hpp"
#include "triwhirl/motor_execution.hpp"
#include "triwhirl/motor_mailbox.hpp"
#include "triwhirl/simplefoc_motor_backend.hpp"
#include "triwhirl/simplefoc_sensor_path.hpp"

extern "C" __attribute__((used, noinline))
void triwhirl_route_b_simplefoc_link_probe() {
  // Compile/link placeholders only. None of the numeric values below are
  // TriWhirl hardware tuning or commissioning authority. They exist solely to
  // make each production-shaped API path type/config-valid so the compiler and
  // linker must resolve the complete architecture. app_main() never calls this
  // function and therefore cannot energize these objects.
  triwhirl::simplefoc::SimpleFocSensorConfig sensor_config{};
  sensor_config.i2c_bus_index = 1;
  sensor_config.sda_gpio = 21;
  sensor_config.scl_gpio = 22;
  sensor_config.i2c_hz = 400000U;
  triwhirl::simplefoc::SimpleFocSensorPath sensor_only(sensor_config);

  // Taking the addresses of the out-of-line passive APIs forces the Route-B
  // linker to resolve their implementations without executing begin/service.
  auto sensor_begin = &triwhirl::simplefoc::SimpleFocSensorPath::begin;
  auto sensor_service = &triwhirl::simplefoc::SimpleFocSensorPath::service;

  triwhirl::simplefoc::SimpleFocMotorBackendConfig backend_config{};
  backend_config.i2c_bus_index = 1;
  backend_config.sda_gpio = 21;
  backend_config.scl_gpio = 22;
  backend_config.i2c_hz = 400000U;
  backend_config.pole_pairs = 1;
  backend_config.pwm_a_gpio = 2;
  backend_config.pwm_b_gpio = 4;
  backend_config.pwm_c_gpio = 16;
  backend_config.supply_voltage_v = 5.0F;
  backend_config.voltage_limit_v = 1.0F;
  backend_config.sensor_align_voltage_v = 0.5F;
  backend_config.target_velocity_limit_rad_s = 10.0F;
  backend_config.velocity_p = 0.1F;
  backend_config.velocity_i = 0.1F;
  backend_config.velocity_d = 0.0F;
  backend_config.velocity_output_ramp = 100.0F;
  backend_config.velocity_lpf_tf_s = 0.02F;

  triwhirl::simplefoc::SimpleFocMotorBackend backend(backend_config);
  triwhirl::MotorControl motor_control = backend.makeControl();

  triwhirl::MotorCommandMailbox command_mailbox;
  triwhirl::MotorObservationMailbox observation_mailbox;

  triwhirl::MotorExecutionConfig execution_config{};
  execution_config.command_timeout_us = 5000U;
  triwhirl::MotorExecutionDomain executor(
      motor_control, &command_mailbox, &observation_mailbox, execution_config);

  triwhirl::runtime::MotorExecutionTaskConfig task_config{};
  task_config.task_name = "sfoc-link-probe";
  task_config.stack_depth = 4096U;
  task_config.priority = configMAX_PRIORITIES - 3;
  task_config.core_id = 0;
  task_config.service_period_us = 1000U;
  task_config.late_slack_us = 100U;
  task_config.release_interrupt_priority = 1;
  triwhirl::runtime::MotorExecutionTask motor_task(
      &executor, &observation_mailbox, task_config);

  constexpr float kProbeWheelScaleRadS = 10.0F;
  constexpr float kProbeTargetLimitRadS = 10.0F;
  triwhirl::FuzzyAttitudeCommandConfig fuzzy_config{};
  fuzzy_config.fuzzy.theta_error_scale_rad = 1.0F;
  fuzzy_config.fuzzy.theta_rate_scale_rad_s = 1.0F;
  fuzzy_config.fuzzy.wheel_velocity_scale_rad_s = kProbeWheelScaleRadS;
  fuzzy_config.fuzzy.target_velocity_limit_rad_s = kProbeTargetLimitRadS;
  fuzzy_config.fuzzy.target_velocity_singletons =
      triwhirl::fuzzy_balance::makeQualitativeRuleSeed(
          kProbeWheelScaleRadS, kProbeTargetLimitRadS);
  fuzzy_config.fuzzy.rule_surface_configured = true;
  fuzzy_config.upright_reference_rad = 0.0F;
  fuzzy_config.max_motor_observation_age_us = 3000U;
  triwhirl::FuzzyAttitudeCommandController fuzzy_controller(fuzzy_config);

  const std::uint32_t observation_generation = observation_mailbox.publish(
      2.0F, 0.0F, 0U, 0U, 1000U,
      true, true, false, false, false);

  triwhirl::MotorObservationSnapshot mechanical_observation{};
  const bool observation_available =
      observation_mailbox.tryRead(&mechanical_observation);

  triwhirl::FuzzyAttitudeCommandInput fuzzy_input{};
  fuzzy_input.theta_rad = 0.02F;
  fuzzy_input.theta_rate_rad_s = 0.0F;
  fuzzy_input.attitude_valid = true;
  fuzzy_input.now_us32 = 1100U;
  fuzzy_input.motor_observation_available = observation_available;
  fuzzy_input.motor = mechanical_observation;

  const triwhirl::FuzzyAttitudeCommandOutput fuzzy_output =
      fuzzy_controller.evaluate(fuzzy_input);
  const std::uint32_t command_generation =
      fuzzy_controller.publish(fuzzy_input, &command_mailbox);

  triwhirl::MotorCommandSnapshot mechanical_command{};
  const bool command_available = command_mailbox.tryRead(&mechanical_command);

  // Validation-only reads: no begin(), no service(), no I2C, no PWM, no FOC
  // service, and no FreeRTOS task creation. The volatile aggregate keeps the
  // complete passive + target path observable to the compiler/linker while it
  // remains unreachable from the real firmware entry point.
  volatile bool stack_valid =
      sensor_only.configValid() && sensor_begin != nullptr &&
      sensor_service != nullptr && backend.configValid() &&
      motor_control.valid() && executor.valid() && motor_task.valid() &&
      fuzzy_controller.valid() && observation_generation != 0U &&
      observation_available && fuzzy_output.target_valid &&
      std::isfinite(fuzzy_output.target_velocity_rad_s) &&
      command_generation != 0U && command_available &&
      mechanical_command.enabled &&
      std::isfinite(mechanical_command.target_velocity_rad_s);
  (void)stack_valid;
}

#endif  // Route-B production-shaped SimpleFOC proof
