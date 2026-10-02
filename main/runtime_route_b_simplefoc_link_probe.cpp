// Issue #53 production-shaped Route-B compile/link proof only.
//
// Native ESP-IDF builds compile this translation unit as empty. The Route-B
// environment enables the production-shaped triwhirl_simplefoc component and
// retains the function below at link time. The function is NEVER called by
// app_main(), so it performs no I2C access, FOC initialization, PWM enable, or
// task creation on a running device.

#if defined(TRIWHIRL_ROUTE_B_SIMPLEFOC_LINK_PROBE) && \
    defined(TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND)

#include <cstdint>

#include "runtime_motor_task.hpp"
#include "triwhirl/motor_execution.hpp"
#include "triwhirl/motor_mailbox.hpp"
#include "triwhirl/simplefoc_motor_backend.hpp"

extern "C" __attribute__((used, noinline))
void triwhirl_route_b_simplefoc_link_probe() {
  // Compile/link placeholders only. None of these values are TriWhirl hardware
  // authority or commissioning data; real values require schematic/datasheet
  // evidence and independent motor/encoder measurement before activation.
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

  // These are validation-only calls. They contain no hardware I/O and create no
  // RTOS task. Referencing the objects forces the linker to resolve the actual
  // production-shaped backend plus execution/scheduling stack.
  volatile bool stack_valid = backend.configValid() && motor_control.valid() &&
                              executor.valid() && motor_task.valid();
  (void)stack_valid;
}

#endif  // Route-B production-shaped SimpleFOC proof
