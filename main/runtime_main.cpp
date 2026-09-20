// Transitional runtime integration for the firmware-owned swing-identification
// experiment.  Keep the established bring-up runtime intact while #31 is
// validated on hardware; the legacy app_main symbol is renamed inside this
// translation unit, and the real app_main below adds the swing supervisor.
#define app_main triwhirl_legacy_app_main
#include "app_main.cpp"
#undef app_main

#include "freertos/queue.h"
#include "triwhirl/swing_id.hpp"

namespace {

using triwhirl::SwingIdConfig;
using triwhirl::SwingIdInput;
using triwhirl::SwingIdOutput;
using triwhirl::SwingIdRunner;
using triwhirl::SwingIdState;
using triwhirl::SwingIdStopReason;
using triwhirl::SwingIdVertex;

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

void resetRuntimeTimingProfile() {
  const bool enabled = runtime_timing_profile.enabled;
  runtime_timing_profile = {};
  runtime_timing_profile.enabled = enabled;
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
    const double mean_us = stats.count > 0U
                               ? static_cast<double>(stats.total_us) /
                                     static_cast<double>(stats.count)
                               : 0.0;
    const std::uint32_t min_us = stats.count > 0U ? stats.min_us : 0U;
    consolePrintf(
        "timing_profile_stage,name=%s,count=%llu,mean_us=%.3f,min_us=%lu,max_us=%lu\r\n",
        runtimeTimingStageName(stage),
        static_cast<unsigned long long>(stats.count), mean_us,
        static_cast<unsigned long>(min_us),
        static_cast<unsigned long>(stats.max_us));
  }
  consoleWrite("timing_profile_end\r\n");
}

void handleRuntimeTimingProfileCommand(char* line) {
  if (line == nullptr) {
    return;
  }
  std::strtok(line, " \t");  // timing
  char* profile = std::strtok(nullptr, " \t");
  char* action = std::strtok(nullptr, " \t");
  if (profile == nullptr || std::strcmp(profile, "profile") != 0) {
    consoleWrite("ERR usage: timing profile <status|on|off|reset>\r\n");
    return;
  }
  if (action == nullptr || std::strcmp(action, "status") == 0) {
    printRuntimeTimingProfile();
    return;
  }
  if (std::strcmp(action, "reset") == 0) {
    resetRuntimeTimingProfile();
    consoleWrite("OK timing profile reset\r\n");
    return;
  }
  if (std::strcmp(action, "on") == 0) {
    runtime_timing_profile.enabled = false;
    resetRuntimeTimingProfile();
    runtime_timing_profile.enabled = true;
    consoleWrite("OK timing profile on\r\n");
    return;
  }
  if (std::strcmp(action, "off") == 0) {
    runtime_timing_profile.enabled = false;
    consoleWrite("OK timing profile off\r\n");
    printRuntimeTimingProfile();
    return;
  }
  consoleWrite("ERR usage: timing profile <status|on|off|reset>\r\n");
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

bool parseSwingConfig(SwingIdConfig* const config) {
  if (config == nullptr) {
    return false;
  }
  char* target = std::strtok(nullptr, " \t");
  char* pump_low = std::strtok(nullptr, " \t");
  char* pump_high = std::strtok(nullptr, " \t");
  char* capture = std::strtok(nullptr, " \t");
  char* probe_exit = std::strtok(nullptr, " \t");
  char* rearm = std::strtok(nullptr, " \t");
  char* probe_ms = std::strtok(nullptr, " \t");
  char* rate_switch = std::strtok(nullptr, " \t");
  char* polarity = std::strtok(nullptr, " \t");
  char* vertex_a = std::strtok(nullptr, " \t");
  char* max_s = std::strtok(nullptr, " \t");
  if (target == nullptr || pump_low == nullptr || pump_high == nullptr ||
      capture == nullptr || probe_exit == nullptr || rearm == nullptr ||
      probe_ms == nullptr || rate_switch == nullptr || polarity == nullptr ||
      vertex_a == nullptr || max_s == nullptr || std::strtok(nullptr, " \t") != nullptr) {
    return false;
  }

  const double probe_ms_value = std::strtod(probe_ms, nullptr);
  const double max_s_value = std::strtod(max_s, nullptr);
  SwingIdConfig candidate{};
  candidate.target_captures =
      static_cast<std::uint32_t>(std::strtoul(target, nullptr, 10));
  candidate.pump_v_low = std::strtof(pump_low, nullptr);
  candidate.pump_v_high = std::strtof(pump_high, nullptr);
  candidate.capture_deg = std::strtof(capture, nullptr);
  candidate.probe_exit_deg = std::strtof(probe_exit, nullptr);
  candidate.rearm_deg = std::strtof(rearm, nullptr);
  candidate.rate_switch_rad_s = std::strtof(rate_switch, nullptr);
  candidate.pump_polarity = std::atoi(polarity);
  candidate.vertex_a_deg = std::strtof(vertex_a, nullptr);

  if (!std::isfinite(probe_ms_value) || probe_ms_value <= 0.0 ||
      probe_ms_value > static_cast<double>(std::numeric_limits<std::uint32_t>::max()) * 1.0e-3 ||
      !std::isfinite(max_s_value) || max_s_value <= 0.0 ||
      max_s_value > static_cast<double>(std::numeric_limits<std::uint32_t>::max()) * 1.0e-6) {
    return false;
  }
  candidate.probe_duration_us =
      static_cast<std::uint32_t>(std::llround(probe_ms_value * 1000.0));
  candidate.max_duration_us =
      static_cast<std::uint32_t>(std::llround(max_s_value * 1000000.0));
  if (candidate.pump_v_high > kMotorVectorLimitV ||
      candidate.pump_v_low > kMotorVectorLimitV) {
    return false;
  }
  *config = candidate;
  return true;
}

void handleSwingCommand(char* line) {
  std::strtok(line, " \t");  // consume "swing"
  char* action = std::strtok(nullptr, " \t");
  if (action == nullptr || std::strcmp(action, "status") == 0) {
    printSwingStatus();
    return;
  }

  if (std::strcmp(action, "config") == 0) {
    SwingIdConfig config{};
    if (!parseSwingConfig(&config) || !swing_id_runner.configure(config)) {
      consoleWrite(
          "ERR usage: swing config <captures> <pump_low_v> <pump_high_v> <capture_deg> <exit_deg> <rearm_deg> <probe_ms> <rate_switch_rad_s> <polarity> <vertex_a_deg> <max_s>\r\n");
      return;
    }
    consoleWrite("OK swing config\r\n");
    printSwingStatus();
    return;
  }

  if (std::strcmp(action, "start") == 0) {
    if (swing_id_runner.active()) {
      consoleWrite("ERR swing already active\r\n");
      return;
    }
    if (motorActive()) {
      consoleWrite("ERR swing start requires motor stopped\r\n");
      return;
    }
    if (!motor_config_valid || !encoder_sample_valid ||
        !wheel_state.velocity_valid || !imu_sample_valid || !gyro_bias_valid ||
        !attitude_state.valid || safety_latch.faulted()) {
      consoleWrite(
          "ERR swing start requires motor config, encoder/wheel, calibrated IMU, valid attitude, and clear safety\r\n");
      return;
    }
    const LoggerStatus log_status = runtime_logger.status();
    const std::uint32_t needed_records =
        (swing_id_runner.config().max_duration_us +
         triwhirl::log::kTwLogSamplePeriodUs - 1U) /
        triwhirl::log::kTwLogSamplePeriodUs;
    if (log_status.state != triwhirl::log::LoggerState::kRecording ||
        log_status.max_records < needed_records) {
      consoleWrite(
          "ERR swing start requires active TWLG recording with capacity for max duration\r\n");
      return;
    }

    const std::uint32_t now_us = static_cast<std::uint32_t>(esp_timer_get_time());
    if (!swing_id_runner.start(currentSwingInput(now_us))) {
      consoleWrite("ERR swing start rejected\r\n");
      return;
    }
    setSwingCriticalWindow(false);
    vq_command_v = clampFinite(swing_id_runner.output().desired_vq_v,
                               -kMotorVectorLimitV, kMotorVectorLimitV);
    motor_mode = MotorMode::kFoc;
    consoleWrite("OK swing start\r\n");
    queueSwingEvent(swing_id_runner.output());
    return;
  }

  if (std::strcmp(action, "abort") == 0) {
    if (!swing_id_runner.active()) {
      consoleWrite("OK swing already inactive\r\n");
      return;
    }
    const SwingIdOutput output =
        swing_id_runner.abort(SwingIdStopReason::kExternalAbort);
    consoleWrite("OK swing abort\r\n");
    finishSwingRun(output);
    return;
  }

  consoleWrite("ERR usage: swing <status|config ...|start|abort>\r\n");
}

bool commandAllowedDuringSwing(const char* line) {
  if (!swing_id_runner.active() || line == nullptr) {
    return true;
  }
  while (*line == ' ' || *line == '\t') {
    ++line;
  }
  if (std::strncmp(line, "swing", 5) == 0 &&
      (line[5] == '\0' || line[5] == ' ' || line[5] == '\t')) {
    return true;
  }
  static const char* const kReadOnlyPrefixes[] = {
      "status", "timing status", "imu status", "attitude status",
      "fault status", "ble status", "log status", "telemetry off", "help",
  };
  for (const char* prefix : kReadOnlyPrefixes) {
    const std::size_t length = std::strlen(prefix);
    if (std::strncmp(line, prefix, length) == 0 &&
        (line[length] == '\0' || line[length] == ' ' || line[length] == '\t')) {
      return true;
    }
  }
  return false;
}

void handleSupervisorCommand(char* line) {
  if (line == nullptr) {
    return;
  }
  char* begin = line;
  while (*begin == ' ' || *begin == '\t') {
    ++begin;
  }
  if (std::strncmp(begin, "swing", 5) == 0 &&
      (begin[5] == '\0' || begin[5] == ' ' || begin[5] == '\t')) {
    handleSwingCommand(begin);
    return;
  }
  if (std::strncmp(begin, "timing profile", 14) == 0 &&
      (begin[14] == '\0' || begin[14] == ' ' || begin[14] == '\t')) {
    handleRuntimeTimingProfileCommand(begin);
    return;
  }
  if (!commandAllowedDuringSwing(begin)) {
    consoleWrite("ERR swing experiment owns realtime actuation; use 'swing abort' first\r\n");
    return;
  }
  const bool help = std::strcmp(begin, "help") == 0;
  handleCommand(begin);
  if (help) {
    printSwingHelp();
  }
}

void consumeSupervisorBytes(const std::uint8_t* input,
                            const std::size_t received,
                            CommandInputState& state) {
  if (input == nullptr) {
    return;
  }
  for (std::size_t index = 0; index < received; ++index) {
    const char c = static_cast<char>(input[index]);
    if (c == '\r' || c == '\n') {
      if (state.length > 0U) {
        consoleWrite("\r\n");
        state.line[state.length] = '\0';
        handleSupervisorCommand(state.line);
        state.length = 0U;
        if (!swing_id_runner.active()) {
          printPrompt();
        }
      }
      continue;
    }
    if (c == '\b' || static_cast<unsigned char>(c) == 0x7FU) {
      if (state.length > 0U) {
        --state.length;
        consoleWrite("\b \b");
      }
      continue;
    }
    if (c < 0x20 || static_cast<unsigned char>(c) > 0x7EU) {
      continue;
    }
    if (state.length + 1U < sizeof(state.line)) {
      state.line[state.length++] = c;
      consoleWriteBytes(&c, 1U);
    } else {
      state.length = 0U;
      consoleWrite("\r\nERR command too long\r\n");
      if (!swing_id_runner.active()) {
        printPrompt();
      }
    }
  }
}

void pollSupervisorConsole() {
  std::uint8_t input[64];
  const int uart_received = uart_read_bytes(UART_NUM_0, input, sizeof(input), 0);
  if (uart_received > 0) {
    consumeSupervisorBytes(input, static_cast<std::size_t>(uart_received),
                           uart_command_input);
  }
  const std::size_t ble_received = triwhirl::ble::read(input, sizeof(input));
  if (ble_received > 0U) {
    consumeSupervisorBytes(input, ble_received, ble_command_input);
  }
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

void swingControlTask(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  while (true) {
    const std::int64_t start_us = esp_timer_get_time();
    const std::uint32_t loop_us = static_cast<std::uint32_t>(start_us);
    const bool profile = runtime_timing_profile.enabled;
    std::int64_t stage_us = start_us;

    updateEncoder(loop_us);
    if (profile) {
      const std::int64_t now = esp_timer_get_time();
      recordRuntimeTimingStage(RuntimeTimingStage::kEncoder, stage_us, now);
      stage_us = now;
    }

    updateImu(loop_us);
    if (profile) {
      const std::int64_t now = esp_timer_get_time();
      recordRuntimeTimingStage(RuntimeTimingStage::kImuAttitude, stage_us, now);
      stage_us = now;
    }

    evaluateSafety(start_us);
    updateSwingIdentification(loop_us);
    if (profile) {
      const std::int64_t now = esp_timer_get_time();
      recordRuntimeTimingStage(RuntimeTimingStage::kSafetySwing, stage_us, now);
      stage_us = now;
    }

    updateMotor(loop_us);
    if (profile) {
      const std::int64_t now = esp_timer_get_time();
      recordRuntimeTimingStage(RuntimeTimingStage::kMotor, stage_us, now);
      stage_us = now;
    }

    recordSwingRuntimeLog(loop_us);
    finalizeSwingLogIfPending();
    if (profile) {
      const std::int64_t now = esp_timer_get_time();
      recordRuntimeTimingStage(RuntimeTimingStage::kLog, stage_us, now);
      stage_us = now;
    }

    pollSupervisorConsole();
    finalizeSwingLogIfPending();
    if (profile) {
      const std::int64_t now = esp_timer_get_time();
      recordRuntimeTimingStage(RuntimeTimingStage::kConsole, stage_us, now);
      stage_us = now;
    }

    emitTelemetry(loop_us);
    const std::int64_t end_us = esp_timer_get_time();
    if (profile) {
      recordRuntimeTimingStage(RuntimeTimingStage::kTelemetry, stage_us, end_us);
      recordRuntimeTimingStage(RuntimeTimingStage::kLoop, start_us, end_us);
    }
    updateTimingStats(start_us, end_us);
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
  }
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

  if (xTaskCreatePinnedToCore(swingControlTask, "triwhirl_control", 8192, nullptr,
                              configMAX_PRIORITIES - 2, nullptr, 1) != pdPASS) {
    safety_latch.trip(SafetyFault::kStartup);
    stopMotor();
    consoleWrite("FATAL fault=startup control task creation failed\r\n");
    return;
  }

  vTaskDelete(nullptr);
}
