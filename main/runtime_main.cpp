// Transitional runtime integration for the firmware-owned swing-identification
// experiment. The remaining bridge exposes the established bring-up state to
// runtime_control while startup/state ownership is extracted explicitly.
#define app_main triwhirl_legacy_app_main
#define initEncoderBus triwhirl_legacy_initEncoderBus
#define initImuBus triwhirl_legacy_initImuBus
#include "app_main.cpp"
#undef initImuBus
#undef initEncoderBus
#undef app_main

#include "freertos/queue.h"
#include "runtime_control.hpp"
#include "runtime_platform.hpp"
#include "triwhirl/swing_id.hpp"

namespace {

using triwhirl::SwingIdConfig;
using triwhirl::SwingIdInput;
using triwhirl::SwingIdOutput;
using triwhirl::SwingIdRunner;
using triwhirl::SwingIdState;
using triwhirl::SwingIdStopReason;
using triwhirl::SwingIdVertex;

bool initEncoderBus(i2c_master_bus_handle_t* bus) {
  i2c_master_bus_config_t config{};
  config.i2c_port = I2C_NUM_0;
  config.sda_io_num = static_cast<gpio_num_t>(triwhirl::board::kAs5600SdaGpio);
  config.scl_io_num = static_cast<gpio_num_t>(triwhirl::board::kAs5600SclGpio);
  config.clk_source = I2C_CLK_SRC_DEFAULT;
  config.glitch_ignore_cnt = 7;
  config.flags.enable_internal_pullup = true;
  return triwhirl::runtime::createI2cMasterBusOnCore(&config, bus, 0) == ESP_OK;
}

bool initImuBus(i2c_master_bus_handle_t* bus) {
  i2c_master_bus_config_t config{};
  config.i2c_port = I2C_NUM_1;
  config.sda_io_num = static_cast<gpio_num_t>(triwhirl::board::kMpu6050SdaGpio);
  config.scl_io_num = static_cast<gpio_num_t>(triwhirl::board::kMpu6050SclGpio);
  config.clk_source = I2C_CLK_SRC_DEFAULT;
  config.glitch_ignore_cnt = 7;
  config.flags.enable_internal_pullup = true;
  return triwhirl::runtime::createI2cMasterBusOnCore(&config, bus, 1) == ESP_OK;
}

struct SwingEvent {
  SwingIdOutput output{};
  std::uint32_t target_captures = 0U;
  std::uint32_t fault_mask = 0U;
};

enum class RuntimeTimingStage : std::uint8_t {
  kEncoder = 0,
  kImuAttitude,
  kSafetySwing,
  kMotor,
  kLog,
  kConsole,
  kTelemetry,
  kLoop,
  kCount,
};

struct RuntimeTimingStageStats {
  std::uint64_t count = 0U;
  std::uint64_t total_us = 0U;
  std::uint32_t min_us = std::numeric_limits<std::uint32_t>::max();
  std::uint32_t max_us = 0U;
};

struct RuntimeTimingProfile {
  bool enabled = false;
  RuntimeTimingStageStats stages[static_cast<std::size_t>(RuntimeTimingStage::kCount)]{};
};

SwingIdRunner swing_id_runner{};
QueueHandle_t swing_event_queue = nullptr;
std::uint32_t swing_event_drops = 0U;
bool swing_log_finalize_pending = false;
RuntimeTimingProfile runtime_timing_profile{};

void printSwingHelp();

const char* runtimeTimingStageName(const RuntimeTimingStage stage) {
  switch (stage) {
    case RuntimeTimingStage::kEncoder:
      return "encoder";
    case RuntimeTimingStage::kImuAttitude:
      return "imu_attitude";
    case RuntimeTimingStage::kSafetySwing:
      return "safety_swing";
    case RuntimeTimingStage::kMotor:
      return "motor";
    case RuntimeTimingStage::kLog:
      return "log";
    case RuntimeTimingStage::kConsole:
      return "console";
    case RuntimeTimingStage::kTelemetry:
      return "telemetry";
    case RuntimeTimingStage::kLoop:
      return "loop";
    case RuntimeTimingStage::kCount:
      break;
  }
  return "unknown";
}

void printTimingProfileStage(const char* const name,
                             const std::uint64_t count,
                             const std::uint64_t total_us,
                             const std::uint32_t min_us,
                             const std::uint32_t max_us) {
  const double mean_us = count > 0U
                             ? static_cast<double>(total_us) /
                                   static_cast<double>(count)
                             : 0.0;
  consolePrintf(
      "timing_profile_stage,name=%s,count=%llu,mean_us=%.3f,min_us=%lu,max_us=%lu\r\n",
      name, static_cast<unsigned long long>(count), mean_us,
      static_cast<unsigned long>(count > 0U ? min_us : 0U),
      static_cast<unsigned long>(count > 0U ? max_us : 0U));
}

void resetRuntimeTimingProfile() {
  const bool enabled = runtime_timing_profile.enabled;
  runtime_timing_profile = {};
  runtime_timing_profile.enabled = enabled;
  encoder.resetTimingProfile();
  imu.resetTimingProfile();
}

void recordRuntimeTimingStage(const RuntimeTimingStage stage,
                              const std::int64_t begin_us,
                              const std::int64_t end_us) {
  if (!runtime_timing_profile.enabled || end_us < begin_us ||
      stage == RuntimeTimingStage::kCount) {
    return;
  }
  const std::uint64_t elapsed64 = static_cast<std::uint64_t>(end_us - begin_us);
  const std::uint32_t elapsed = elapsed64 > std::numeric_limits<std::uint32_t>::max()
                                    ? std::numeric_limits<std::uint32_t>::max()
                                    : static_cast<std::uint32_t>(elapsed64);
  RuntimeTimingStageStats& stats =
      runtime_timing_profile.stages[static_cast<std::size_t>(stage)];
  ++stats.count;
  stats.total_us += elapsed;
  if (elapsed < stats.min_us) {
    stats.min_us = elapsed;
  }
  if (elapsed > stats.max_us) {
    stats.max_us = elapsed;
  }
}

void printRuntimeTimingProfile() {
  const RuntimeTimingStageStats& loop =
      runtime_timing_profile.stages[static_cast<std::size_t>(RuntimeTimingStage::kLoop)];
  consolePrintf("timing_profile,enabled=%d,samples=%llu,clock=esp_timer_us\r\n",
                runtime_timing_profile.enabled ? 1 : 0,
                static_cast<unsigned long long>(loop.count));
  for (std::size_t index = 0U;
       index < static_cast<std::size_t>(RuntimeTimingStage::kCount); ++index) {
    const RuntimeTimingStage stage = static_cast<RuntimeTimingStage>(index);
    const RuntimeTimingStageStats& stats = runtime_timing_profile.stages[index];
    printTimingProfileStage(runtimeTimingStageName(stage), stats.count,
                            stats.total_us, stats.min_us, stats.max_us);
  }

  const auto encoder_timing = encoder.timingProfile();
  printTimingProfileStage("encoder_i2c_raw", encoder_timing.raw_reads,
                          encoder_timing.raw_total_us,
                          encoder_timing.raw_min_us,
                          encoder_timing.raw_max_us);
  printTimingProfileStage("encoder_i2c_status", encoder_timing.status_reads,
                          encoder_timing.status_total_us,
                          encoder_timing.status_min_us,
                          encoder_timing.status_max_us);

  const auto imu_timing = imu.timingProfile();
  printTimingProfileStage("mpu_i2c", imu_timing.sample_reads,
                          imu_timing.transfer_total_us,
                          imu_timing.transfer_min_us,
                          imu_timing.transfer_max_us);
  printTimingProfileStage("mpu_decode", imu_timing.sample_reads,
                          imu_timing.decode_total_us,
                          imu_timing.decode_min_us,
                          imu_timing.decode_max_us);
  consoleWrite("timing_profile_end\r\n");
}

bool swingTerminal(const SwingIdState state) {
  return state == SwingIdState::kComplete || state == SwingIdState::kAborted;
}

void queueSwingEvent(const SwingIdOutput& output) {
  if (!output.transition || swing_event_queue == nullptr) {
    return;
  }
  SwingEvent event{};
  event.output = output;
  event.target_captures = swing_id_runner.config().target_captures;
  event.fault_mask = safety_latch.mask();
  if (xQueueSend(swing_event_queue, &event, 0) != pdTRUE) {
    ++swing_event_drops;
  }
}

void swingEventTask(void*) {
  SwingEvent event{};
  while (true) {
    if (xQueueReceive(swing_event_queue, &event, portMAX_DELAY) != pdTRUE) {
      continue;
    }
    const SwingIdOutput& output = event.output;
    consolePrintf(
        "event,swing_id,state=%s,captures=%lu,target=%lu,half_cycle=%lu,vertex=%s,error_deg=%.3f,vq_v=%.3f,reason=%s,fault_mask=0x%08lx\r\n",
        triwhirl::swingIdStateName(output.state),
        static_cast<unsigned long>(output.capture_count),
        static_cast<unsigned long>(event.target_captures),
        static_cast<unsigned long>(output.half_cycle_index),
        triwhirl::swingIdVertexName(output.vertex), output.vertex_error_deg,
        output.desired_vq_v, triwhirl::swingIdStopReasonName(output.stop_reason),
        static_cast<unsigned long>(event.fault_mask));
  }
}

void setSwingCriticalWindow(const bool critical) {
  if (log_critical_window == critical) {
    return;
  }
  log_critical_window = critical;
  runtime_logger.setFlashWritesAllowed(!critical);
}

void finishSwingRun(const SwingIdOutput& output) {
  setSwingCriticalWindow(false);
  stopMotor();
  swing_log_finalize_pending = true;
  queueSwingEvent(output);
}

void applySwingOutput(const SwingIdOutput& output) {
  setSwingCriticalWindow(output.critical_window);
  if (swing_id_runner.active()) {
    vq_command_v = clampFinite(output.desired_vq_v, -kMotorVectorLimitV,
                               kMotorVectorLimitV);
    motor_mode = MotorMode::kFoc;
  } else if (swingTerminal(output.state)) {
    finishSwingRun(output);
    return;
  }
  queueSwingEvent(output);
}

SwingIdInput currentSwingInput(const std::uint32_t now_us) {
  SwingIdInput input{};
  input.now_us = now_us;
  input.theta_rad = attitude_state.angle_rad;
  input.theta_rate_rad_s = attitude_state.rate_rad_s;
  input.attitude_valid = attitude_state.valid && imu_sample_valid && gyro_bias_valid;
  input.safety_faulted = safety_latch.faulted();
  return input;
}

void updateSwingIdentification(const std::uint32_t now_us) {
  if (!swing_id_runner.active()) {
    return;
  }
  const SwingIdOutput output = swing_id_runner.update(currentSwingInput(now_us));
  applySwingOutput(output);
}

void finalizeSwingLogIfPending() {
  if (!swing_log_finalize_pending) {
    return;
  }
  setSwingCriticalWindow(false);
  runtime_logger.stop();
  swing_log_finalize_pending = false;
}

void printSwingStatus() {
  const SwingIdOutput& output = swing_id_runner.output();
  const SwingIdConfig& config = swing_id_runner.config();
  consolePrintf(
      "swing,state=%s,reason=%s,captures=%lu,target=%lu,half_cycle=%lu,vertex=%s,error_deg=%.3f,vq_v=%.3f,pump=%d,probe=%d,critical=%d,event_drops=%lu,pump_low=%.3f,pump_high=%.3f,capture_deg=%.3f,exit_deg=%.3f,rearm_deg=%.3f,probe_ms=%.3f,rate_switch=%.6f,polarity=%d,vertex_a_deg=%.3f,max_s=%.3f\r\n",
      triwhirl::swingIdStateName(output.state),
      triwhirl::swingIdStopReasonName(output.stop_reason),
      static_cast<unsigned long>(output.capture_count),
      static_cast<unsigned long>(config.target_captures),
      static_cast<unsigned long>(output.half_cycle_index),
      triwhirl::swingIdVertexName(output.vertex), output.vertex_error_deg,
      output.desired_vq_v, output.pump_active ? 1 : 0,
      output.probe_active ? 1 : 0, output.critical_window ? 1 : 0,
      static_cast<unsigned long>(swing_event_drops), config.pump_v_low,
      config.pump_v_high, config.capture_deg, config.probe_exit_deg,
      config.rearm_deg, static_cast<float>(config.probe_duration_us) * 1.0e-3F,
      config.rate_switch_rad_s, config.pump_polarity, config.vertex_a_deg,
      static_cast<float>(config.max_duration_us) * 1.0e-6F);
}

std::uint16_t swingRuntimeLogFlags() {
  std::uint16_t flags = runtimeLogFlags();
  const SwingIdOutput& output = swing_id_runner.output();
  if (!swing_id_runner.active()) {
    return flags;
  }
  if (output.pump_active) {
    flags |= triwhirl::log::kRecordPumpActive;
  }
  if (output.probe_active) {
    flags |= triwhirl::log::kRecordProbeActive;
    switch (output.vertex) {
      case SwingIdVertex::kA:
        flags |= triwhirl::log::kRecordVertexA;
        break;
      case SwingIdVertex::kB:
        flags |= triwhirl::log::kRecordVertexB;
        break;
      case SwingIdVertex::kC:
        flags |= triwhirl::log::kRecordVertexC;
        break;
      case SwingIdVertex::kNone:
        break;
    }
  }
  return flags;
}

void recordSwingRuntimeLog(const std::uint32_t now_us) {
  RuntimeLogRecord record{};
  record.t_us = now_us;
  record.theta_rad = attitude_state.angle_rad;
  record.theta_rate_rad_s = attitude_state.rate_rad_s;
  record.wheel_rate_rad_s = wheel_state.velocity_rad_s;
  record.vq_v = vq_command_v;
  record.accel_weight = attitude_state.accel_weight;
  record.fault_mask = safety_latch.mask();
  record.flags = swingRuntimeLogFlags();
  record.raw_count = wheel_state.raw_count;
  runtime_logger.record(record);
}

void printSwingHelp() {
  consoleWrite("  swing status\r\n");
  consoleWrite("  swing config <captures> <pump_low_v> <pump_high_v> <capture_deg> <exit_deg> <rearm_deg> <probe_ms> <rate_switch_rad_s> <polarity> <vertex_a_deg> <max_s>\r\n");
  consoleWrite("  swing start\r\n");
  consoleWrite("  swing abort\r\n");
  consoleWrite("  timing profile <status|on|off|reset>\r\n");
}

}  // namespace

extern "C" void app_main(void) {
  if (!initConsole()) {
    safety_latch.trip(SafetyFault::kStartup);
    return;
  }

  console_tx_stream = xStreamBufferCreate(kConsoleTxBufferBytes, 1U);
  if (console_tx_stream == nullptr) {
    safety_latch.trip(SafetyFault::kStartup);
    return;
  }
  if (xTaskCreatePinnedToCore(consoleTxTask, "triwhirl_uart_tx", 4096, nullptr,
                              2, nullptr, 0) != pdPASS) {
    safety_latch.trip(SafetyFault::kStartup);
    return;
  }

  swing_event_queue = xQueueCreate(8U, sizeof(SwingEvent));
  if (swing_event_queue == nullptr ||
      xTaskCreatePinnedToCore(swingEventTask, "triwhirl_swing_evt", 4096, nullptr,
                              2, nullptr, 0) != pdPASS) {
    safety_latch.trip(SafetyFault::kStartup);
    return;
  }

  if (!triwhirl::ble::init()) {
    consoleWrite("WARN BLE init failed; Web Bluetooth unavailable\r\n");
  }
  if (!runtime_logger.init()) {
    consoleWrite("WARN TWLG init failed; binary runtime logging unavailable\r\n");
  }

  i2c_master_bus_handle_t encoder_bus = nullptr;
  if (!initEncoderBus(&encoder_bus) ||
      !encoder.init(encoder_bus, triwhirl::board::kAs5600I2cAddress)) {
    safety_latch.trip(SafetyFault::kStartup);
    consoleWrite("FATAL fault=startup AS5600 I2C init failed\r\n");
    return;
  }

  i2c_master_bus_handle_t imu_bus = nullptr;
  imu_ready = initImuBus(&imu_bus) &&
              imu.init(imu_bus, triwhirl::board::kMpu6050I2cAddress);
  if (!imu_ready) {
    consoleWrite("WARN MPU6050 init failed; IMU functions unavailable\r\n");
  }

  if (!bridge.init(triwhirl::board::kMotorIn1Gpio,
                   triwhirl::board::kMotorIn2Gpio,
                   triwhirl::board::kMotorIn3Gpio, kPwmFrequencyHz,
                   triwhirl::board::kMotorBusNominalV)) {
    safety_latch.trip(SafetyFault::kActuator);
    consoleWrite("FATAL fault=actuator MCPWM bridge init failed\r\n");
    return;
  }

  bridge.stopZeroVector();
  const std::uint32_t now_us = static_cast<std::uint32_t>(esp_timer_get_time());
  sampleEncoder(now_us);
  refreshEncoderHealth();
  if (imu_ready) {
    sampleImu();
    startGyroCalibration(kDefaultGyroCalibrationSamples);
  }
  last_motor_update_us = now_us;
  last_encoder_sample_us = now_us;
  last_encoder_health_us = now_us;
  last_imu_sample_us = now_us;
  last_telemetry_us = now_us;

  consoleWrite("TriWhirl deterministic motor + IMU + attitude runtime ready\r\n");
  consoleWrite("ESP32 owns swing-identification realtime decisions; host/BLE is supervisory only\r\n");
  consoleWrite("TWLG is the authoritative 1 kHz identification time base; BLE events are observability only\r\n");
  consoleWrite("telemetry is off by default; use 'telemetry on' only for diagnostic streaming\r\n");
  printStatus();
  printLogStatus();
  printSwingStatus();
  printHelp();
  printSwingHelp();
  printPrompt();

  if (xTaskCreatePinnedToCore(triwhirl::runtime::realtimeControlTask,
                              "triwhirl_control", 8192, nullptr,
                              configMAX_PRIORITIES - 2, nullptr, 1) != pdPASS) {
    safety_latch.trip(SafetyFault::kStartup);
    stopMotor();
    consoleWrite("FATAL fault=startup control task creation failed\r\n");
    return;
  }

  vTaskDelete(nullptr);
}
