#include "runtime_simplefoc_motor_commissioning.hpp"

#if defined(TRIWHIRL_ROUTE_B_MOTOR_COMMISSIONING) && \
    defined(TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND)

#include <Arduino.h>

#include "esp32-hal-bt.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || \
    !defined(CONFIG_BT_CLASSIC_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
#error "SimpleFOC motor commissioning requires Bluedroid + Classic BT + SPP"
#endif

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <new>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "triwhirl/board.hpp"
#include "triwhirl/motor_control.hpp"
#include "triwhirl/simplefoc_motor_backend.hpp"

namespace triwhirl::runtime {
namespace {

constexpr char kBluetoothDeviceName[] = "TriWhirl-Motor";
constexpr char kBluetoothServiceName[] = "TriWhirl Motor Commissioning";
constexpr EventBits_t kBluetoothReadyBit = BIT0;
constexpr EventBits_t kBluetoothFailedBit = BIT1;

constexpr float kMaxCommissioningVoltageV = 0.5F;
constexpr float kMaxTargetVelocityRadS = 5.0F;
constexpr std::uint32_t kMaxDriveDurationMs = 1500U;
constexpr std::uint32_t kInterDirectionStopMs = 1000U;
constexpr std::uint32_t kMotorServicePeriodMs = 1U;
constexpr std::uint32_t kSensorBusHz = 400000U;
constexpr UBaseType_t kMotorTaskPriority = configMAX_PRIORITIES - 3;
constexpr UBaseType_t kCommandTaskPriority = 2U;
constexpr BaseType_t kMotorTaskCore = 0;
constexpr BaseType_t kCommandTaskCore = 1;

EventGroupHandle_t bluetooth_event_group = nullptr;
StreamBufferHandle_t bluetooth_rx_stream = nullptr;
QueueHandle_t command_queue = nullptr;
QueueHandle_t result_queue = nullptr;

std::atomic<std::uint32_t> bluetooth_spp_handle{0U};
std::atomic<bool> bluetooth_spp_congested{false};
std::atomic<bool> bluetooth_spp_tx_pending{false};
std::atomic<bool> abort_requested{false};
std::atomic<bool> motor_busy{false};

struct CommissionCommand {
  int pole_pairs = 0;
  float supply_voltage_v = 0.0F;
  float voltage_limit_v = 0.0F;
  float sensor_align_voltage_v = 0.0F;
  float target_velocity_rad_s = 0.0F;
  float velocity_p = 0.0F;
  float velocity_lpf_tf_s = 0.0F;
  std::uint32_t drive_duration_ms = 0U;
};

struct CommissionResult {
  bool init_ok = false;
  bool aborted = false;
  bool backend_faulted = false;
  bool sensor_valid = false;
  float positive_mean_velocity_rad_s = 0.0F;
  float positive_last_velocity_rad_s = 0.0F;
  float negative_mean_velocity_rad_s = 0.0F;
  float negative_last_velocity_rad_s = 0.0F;
};

[[noreturn]] void fatalLoop(const char* const reason) {
  std::printf("FATAL sfoc_motor reason=%s\r\n", reason);
  std::fflush(stdout);
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void markBluetoothStartupFailed() {
  if (bluetooth_event_group != nullptr) {
    xEventGroupSetBits(bluetooth_event_group, kBluetoothFailedBit);
  }
}

void bluetoothGapCallback(const esp_bt_gap_cb_event_t event,
                          esp_bt_gap_cb_param_t* const param) {
  if (param == nullptr) return;
  switch (event) {
    case ESP_BT_GAP_CFM_REQ_EVT:
      (void)esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
      break;
    case ESP_BT_GAP_PIN_REQ_EVT: {
      esp_bt_pin_code_t pin_code{};
      pin_code[0] = '1';
      pin_code[1] = '2';
      pin_code[2] = '3';
      pin_code[3] = '4';
      (void)esp_bt_gap_pin_reply(param->pin_req.bda, true, 4U, pin_code);
      break;
    }
    default:
      break;
  }
}

void bluetoothSppCallback(const esp_spp_cb_event_t event,
                          esp_spp_cb_param_t* const param) {
  if (param == nullptr) return;
  switch (event) {
    case ESP_SPP_INIT_EVT:
      if (param->init.status != ESP_SPP_SUCCESS ||
          esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0U,
                            kBluetoothServiceName) != ESP_OK) {
        markBluetoothStartupFailed();
      }
      break;
    case ESP_SPP_START_EVT:
      if (param->start.status != ESP_SPP_SUCCESS ||
          esp_bt_gap_set_device_name(kBluetoothDeviceName) != ESP_OK ||
          esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                   ESP_BT_GENERAL_DISCOVERABLE) != ESP_OK) {
        markBluetoothStartupFailed();
      } else if (bluetooth_event_group != nullptr) {
        xEventGroupSetBits(bluetooth_event_group, kBluetoothReadyBit);
      }
      break;
    case ESP_SPP_SRV_OPEN_EVT:
      if (param->srv_open.status == ESP_SPP_SUCCESS) {
        bluetooth_spp_handle.store(param->srv_open.handle,
                                   std::memory_order_relaxed);
        bluetooth_spp_congested.store(false, std::memory_order_relaxed);
        bluetooth_spp_tx_pending.store(false, std::memory_order_relaxed);
        abort_requested.store(false, std::memory_order_relaxed);
      }
      break;
    case ESP_SPP_DATA_IND_EVT:
      if (bluetooth_rx_stream != nullptr && param->data_ind.len > 0) {
        (void)xStreamBufferSend(bluetooth_rx_stream, param->data_ind.data,
                                static_cast<size_t>(param->data_ind.len), 0);
      }
      break;
    case ESP_SPP_CLOSE_EVT:
      if (bluetooth_spp_handle.load(std::memory_order_relaxed) ==
          param->close.handle) {
        bluetooth_spp_handle.store(0U, std::memory_order_relaxed);
        bluetooth_spp_congested.store(false, std::memory_order_relaxed);
        bluetooth_spp_tx_pending.store(false, std::memory_order_relaxed);
        abort_requested.store(true, std::memory_order_relaxed);
      }
      break;
    case ESP_SPP_CONG_EVT:
      if (bluetooth_spp_handle.load(std::memory_order_relaxed) ==
          param->cong.handle) {
        bluetooth_spp_congested.store(param->cong.cong,
                                      std::memory_order_relaxed);
      }
      break;
    case ESP_SPP_WRITE_EVT:
      bluetooth_spp_tx_pending.store(false, std::memory_order_relaxed);
      break;
    default:
      break;
  }
}

bool beginBluetoothSpp() {
  bluetooth_event_group = xEventGroupCreate();
  bluetooth_rx_stream = xStreamBufferCreate(512U, 1U);
  if (bluetooth_event_group == nullptr || bluetooth_rx_stream == nullptr) {
    return false;
  }
  if (!btStarted() && !btStartMode(BT_MODE_CLASSIC_BT)) return false;

  esp_bluedroid_status_t status = esp_bluedroid_get_status();
  if (status == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
    if (esp_bluedroid_init() != ESP_OK) return false;
    status = esp_bluedroid_get_status();
  }
  if (status != ESP_BLUEDROID_STATUS_ENABLED &&
      esp_bluedroid_enable() != ESP_OK) {
    return false;
  }
  if (esp_bt_gap_register_callback(bluetoothGapCallback) != ESP_OK ||
      esp_spp_register_callback(bluetoothSppCallback) != ESP_OK) {
    return false;
  }

  esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
  if (esp_bt_gap_set_security_param(param_type, &iocap, sizeof(iocap)) != ESP_OK) {
    return false;
  }

  esp_spp_cfg_t spp_config = BT_SPP_DEFAULT_CONFIG();
  spp_config.mode = ESP_SPP_MODE_CB;
  if (esp_spp_enhanced_init(&spp_config) != ESP_OK) return false;

  const EventBits_t bits = xEventGroupWaitBits(
      bluetooth_event_group, kBluetoothReadyBit | kBluetoothFailedBit, pdFALSE,
      pdFALSE, pdMS_TO_TICKS(5000));
  return (bits & kBluetoothReadyBit) != 0U &&
         (bits & kBluetoothFailedBit) == 0U;
}

bool sendBluetoothLine(const char* const line) {
  const std::uint32_t handle =
      bluetooth_spp_handle.load(std::memory_order_relaxed);
  if (handle == 0U) return false;

  const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
  while ((bluetooth_spp_congested.load(std::memory_order_relaxed) ||
          bluetooth_spp_tx_pending.load(std::memory_order_relaxed)) &&
         static_cast<std::int32_t>(deadline - xTaskGetTickCount()) > 0) {
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  if (bluetooth_spp_congested.load(std::memory_order_relaxed) ||
      bluetooth_spp_tx_pending.exchange(true, std::memory_order_relaxed)) {
    return false;
  }

  const int length = static_cast<int>(std::strlen(line));
  const esp_err_t result = esp_spp_write(
      handle, length,
      reinterpret_cast<std::uint8_t*>(const_cast<char*>(line)));
  if (result != ESP_OK) {
    bluetooth_spp_tx_pending.store(false, std::memory_order_relaxed);
    return false;
  }
  return true;
}

bool validCommand(const CommissionCommand& command) {
  const bool finite =
      std::isfinite(command.supply_voltage_v) &&
      std::isfinite(command.voltage_limit_v) &&
      std::isfinite(command.sensor_align_voltage_v) &&
      std::isfinite(command.target_velocity_rad_s) &&
      std::isfinite(command.velocity_p) &&
      std::isfinite(command.velocity_lpf_tf_s);
  return finite && command.pole_pairs == board::kMotorPolePairs &&
         command.supply_voltage_v > 0.0F && command.supply_voltage_v <= 20.0F &&
         command.voltage_limit_v > 0.0F &&
         command.voltage_limit_v <= kMaxCommissioningVoltageV &&
         command.voltage_limit_v <= command.supply_voltage_v &&
         command.sensor_align_voltage_v > 0.0F &&
         command.sensor_align_voltage_v <= kMaxCommissioningVoltageV &&
         command.sensor_align_voltage_v <= command.supply_voltage_v &&
         std::fabs(command.target_velocity_rad_s) > 0.0F &&
         std::fabs(command.target_velocity_rad_s) <= kMaxTargetVelocityRadS &&
         command.velocity_p >= 0.0F && command.velocity_p <= 1.0F &&
         command.velocity_lpf_tf_s > 0.0F && command.velocity_lpf_tf_s <= 0.1F &&
         command.drive_duration_ms > 0U &&
         command.drive_duration_ms <= kMaxDriveDurationMs;
}

simplefoc::SimpleFocMotorBackendConfig backendConfig(
    const CommissionCommand& command) {
  simplefoc::SimpleFocMotorBackendConfig config{};
  config.i2c_bus_index = 0;
  config.sda_gpio = board::kAs5600SdaGpio;
  config.scl_gpio = board::kAs5600SclGpio;
  config.i2c_hz = kSensorBusHz;
  config.pole_pairs = command.pole_pairs;
  config.pwm_a_gpio = board::kMotorPhaseAGpio;
  config.pwm_b_gpio = board::kMotorPhaseBGpio;
  config.pwm_c_gpio = board::kMotorPhaseCGpio;
  config.supply_voltage_v = command.supply_voltage_v;
  config.voltage_limit_v = command.voltage_limit_v;
  config.sensor_align_voltage_v = command.sensor_align_voltage_v;
  config.target_velocity_limit_rad_s = std::fabs(command.target_velocity_rad_s);
  config.velocity_p = command.velocity_p;
  config.velocity_i = 0.0F;
  config.velocity_d = 0.0F;
  config.velocity_output_ramp = 0.0F;
  config.velocity_lpf_tf_s = command.velocity_lpf_tf_s;
  return config;
}

bool serviceForDuration(const MotorControl& control, const float target_velocity,
                        const std::uint32_t duration_ms, float* const mean_velocity,
                        float* const last_velocity) {
  if (!control.commandTargetVelocityRadS(target_velocity)) return false;

  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t end_tick = last_wake + pdMS_TO_TICKS(duration_ms);
  double velocity_sum = 0.0;
  std::uint32_t velocity_count = 0U;
  float latest_velocity = 0.0F;

  while (static_cast<std::int32_t>(end_tick - xTaskGetTickCount()) > 0) {
    if (abort_requested.load(std::memory_order_relaxed)) {
      control.stop();
      control.serviceBackend();
      return false;
    }
    control.serviceBackend();
    const MotorControlObservation observation = control.observation();
    if (observation.backend_faulted || !observation.sensor_valid) {
      control.stop();
      control.serviceBackend();
      return false;
    }
    latest_velocity = observation.shaft_velocity_rad_s;
    if (static_cast<std::int32_t>(end_tick - xTaskGetTickCount()) <=
        static_cast<std::int32_t>(pdMS_TO_TICKS(duration_ms / 2U))) {
      velocity_sum += latest_velocity;
      ++velocity_count;
    }
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kMotorServicePeriodMs));
  }

  control.stop();
  control.serviceBackend();
  *last_velocity = latest_velocity;
  *mean_velocity = velocity_count == 0U
                       ? latest_velocity
                       : static_cast<float>(velocity_sum /
                                            static_cast<double>(velocity_count));
  return true;
}

void serviceStopped(const MotorControl& control, const std::uint32_t duration_ms) {
  control.stop();
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t end_tick = last_wake + pdMS_TO_TICKS(duration_ms);
  while (static_cast<std::int32_t>(end_tick - xTaskGetTickCount()) > 0) {
    control.serviceBackend();
    if (abort_requested.load(std::memory_order_relaxed)) break;
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kMotorServicePeriodMs));
  }
  control.stop();
  control.serviceBackend();
}

void motorTask(void*) {
  for (;;) {
    CommissionCommand command{};
    if (xQueueReceive(command_queue, &command, portMAX_DELAY) != pdTRUE) continue;

    motor_busy.store(true, std::memory_order_relaxed);
    abort_requested.store(false, std::memory_order_relaxed);
    CommissionResult result{};

    const auto config = backendConfig(command);
    if (simplefoc::validSimpleFocMotorBackendConfig(config)) {
      std::unique_ptr<simplefoc::SimpleFocMotorBackend> backend(
          new (std::nothrow) simplefoc::SimpleFocMotorBackend(config));
      if (backend) {
        const MotorControl control = backend->makeControl();
        result.init_ok = control.begin();
        if (result.init_ok) {
          control.serviceBackend();
          const float speed = std::fabs(command.target_velocity_rad_s);
          bool positive_ok = serviceForDuration(
              control, speed, command.drive_duration_ms,
              &result.positive_mean_velocity_rad_s,
              &result.positive_last_velocity_rad_s);
          serviceStopped(control, kInterDirectionStopMs);
          bool negative_ok = false;
          if (positive_ok && !abort_requested.load(std::memory_order_relaxed)) {
            negative_ok = serviceForDuration(
                control, -speed, command.drive_duration_ms,
                &result.negative_mean_velocity_rad_s,
                &result.negative_last_velocity_rad_s);
          }
          control.stop();
          control.serviceBackend();
          const MotorControlObservation observation = control.observation();
          result.backend_faulted = observation.backend_faulted;
          result.sensor_valid = observation.sensor_valid;
          result.aborted = abort_requested.load(std::memory_order_relaxed) ||
                           !positive_ok || !negative_ok;
        }
      }
    }

    motor_busy.store(false, std::memory_order_relaxed);
    (void)xQueueOverwrite(result_queue, &result);
  }
}

bool parseCommissionCommand(const char* const line, CommissionCommand* const command) {
  unsigned duration_ms = 0U;
  CommissionCommand parsed{};
  const int fields = std::sscanf(
      line, "commission %d %f %f %f %f %f %f %u",
      &parsed.pole_pairs, &parsed.supply_voltage_v, &parsed.voltage_limit_v,
      &parsed.sensor_align_voltage_v, &parsed.target_velocity_rad_s,
      &parsed.velocity_p, &parsed.velocity_lpf_tf_s, &duration_ms);
  if (fields != 8) return false;
  parsed.drive_duration_ms = static_cast<std::uint32_t>(duration_ms);
  if (!validCommand(parsed)) return false;
  *command = parsed;
  return true;
}

void emitResult(const CommissionResult& result) {
  char line[384]{};
  const int written = std::snprintf(
      line, sizeof(line),
      "sfoc_motor_result,init_ok=%u,aborted=%u,backend_faulted=%u,"
      "sensor_valid=%u,pos_mean_rad_s=%.6f,pos_last_rad_s=%.6f,"
      "neg_mean_rad_s=%.6f,neg_last_rad_s=%.6f\r\n",
      result.init_ok ? 1U : 0U, result.aborted ? 1U : 0U,
      result.backend_faulted ? 1U : 0U, result.sensor_valid ? 1U : 0U,
      static_cast<double>(result.positive_mean_velocity_rad_s),
      static_cast<double>(result.positive_last_velocity_rad_s),
      static_cast<double>(result.negative_mean_velocity_rad_s),
      static_cast<double>(result.negative_last_velocity_rad_s));
  if (written > 0 && static_cast<std::size_t>(written) < sizeof(line)) {
    (void)sendBluetoothLine(line);
    std::printf("%s", line);
    std::fflush(stdout);
  }
}

void processLine(char* const line) {
  if (std::strcmp(line, "stop") == 0) {
    abort_requested.store(true, std::memory_order_relaxed);
    (void)sendBluetoothLine("OK sfoc_motor stop_requested\r\n");
    return;
  }
  if (std::strcmp(line, "status") == 0) {
    char response[160]{};
    std::snprintf(response, sizeof(response),
                  "sfoc_motor_status,busy=%u,connected=%u,max_voltage_v=%.3f,"
                  "max_target_rad_s=%.3f,max_duration_ms=%u\r\n",
                  motor_busy.load(std::memory_order_relaxed) ? 1U : 0U,
                  bluetooth_spp_handle.load(std::memory_order_relaxed) != 0U ? 1U : 0U,
                  static_cast<double>(kMaxCommissioningVoltageV),
                  static_cast<double>(kMaxTargetVelocityRadS),
                  static_cast<unsigned>(kMaxDriveDurationMs));
    (void)sendBluetoothLine(response);
    return;
  }

  CommissionCommand command{};
  if (!parseCommissionCommand(line, &command)) {
    (void)sendBluetoothLine(
        "ERR sfoc_motor expected='commission <pp> <supply_v> <limit_v> "
        "<align_v> <speed_rad_s> <P> <Tf_s> <duration_ms>'\r\n");
    return;
  }
  if (motor_busy.load(std::memory_order_relaxed)) {
    (void)sendBluetoothLine("ERR sfoc_motor busy\r\n");
    return;
  }
  if (xQueueSend(command_queue, &command, 0) != pdTRUE) {
    (void)sendBluetoothLine("ERR sfoc_motor queue_full\r\n");
    return;
  }

  char response[256]{};
  std::snprintf(
      response, sizeof(response),
      "OK sfoc_motor accepted,pole_pairs=%d,supply_v=%.3f,limit_v=%.3f,"
      "align_v=%.3f,target_rad_s=%.3f,P=%.6f,I=0,D=0,ramp=0,Tf_s=%.6f,"
      "duration_ms=%u\r\n",
      command.pole_pairs, static_cast<double>(command.supply_voltage_v),
      static_cast<double>(command.voltage_limit_v),
      static_cast<double>(command.sensor_align_voltage_v),
      static_cast<double>(std::fabs(command.target_velocity_rad_s)),
      static_cast<double>(command.velocity_p),
      static_cast<double>(command.velocity_lpf_tf_s),
      static_cast<unsigned>(command.drive_duration_ms));
  (void)sendBluetoothLine(response);
}

void commandTask(void*) {
  char line[256]{};
  std::size_t used = 0U;
  for (;;) {
    CommissionResult result{};
    if (xQueueReceive(result_queue, &result, 0) == pdTRUE) {
      emitResult(result);
    }

    std::uint8_t byte = 0U;
    if (xStreamBufferReceive(bluetooth_rx_stream, &byte, 1U,
                             pdMS_TO_TICKS(20)) != 1U) {
      continue;
    }
    if (byte == '\r' || byte == '\n') {
      if (used > 0U) {
        line[used] = '\0';
        processLine(line);
        used = 0U;
      }
      continue;
    }
    if (used + 1U < sizeof(line)) {
      line[used++] = static_cast<char>(byte);
    } else {
      used = 0U;
      (void)sendBluetoothLine("ERR sfoc_motor line_too_long\r\n");
    }
  }
}

}  // namespace

[[noreturn]] void runSimpleFocMotorCommissioning() {
  initArduino();

  command_queue = xQueueCreate(1U, sizeof(CommissionCommand));
  result_queue = xQueueCreate(1U, sizeof(CommissionResult));
  if (command_queue == nullptr || result_queue == nullptr) {
    fatalLoop("queue_create_failed");
  }
  if (!beginBluetoothSpp()) {
    fatalLoop("bluetooth_spp_begin_failed");
  }

  std::printf(
      "TriWhirl SimpleFOC motor-only commissioning\r\n"
      "transport=bluetooth_spp_direct,name=%s,owner=SimpleFOC,"
      "actuator=DISABLED,autostart=0\r\n"
      "pins=A%d/B%d/C%d,as5600_sda=%d,as5600_scl=%d,pole_pairs_required=%d\r\n"
      "hard_caps=voltage<=%.3fV,target<=%.3frad/s,duration<=%ums\r\n",
      kBluetoothDeviceName, board::kMotorPhaseAGpio, board::kMotorPhaseBGpio,
      board::kMotorPhaseCGpio, board::kAs5600SdaGpio, board::kAs5600SclGpio,
      board::kMotorPolePairs, static_cast<double>(kMaxCommissioningVoltageV),
      static_cast<double>(kMaxTargetVelocityRadS),
      static_cast<unsigned>(kMaxDriveDurationMs));
  std::fflush(stdout);

  if (xTaskCreatePinnedToCore(commandTask, "sfoc_motor_cmd", 6144, nullptr,
                              kCommandTaskPriority, nullptr,
                              kCommandTaskCore) != pdPASS) {
    fatalLoop("command_task_create_failed");
  }
  if (xTaskCreatePinnedToCore(motorTask, "sfoc_motor", 8192, nullptr,
                              kMotorTaskPriority, nullptr,
                              kMotorTaskCore) != pdPASS) {
    fatalLoop("motor_task_create_failed");
  }

  vTaskDelete(nullptr);
  for (;;) {
  }
}

}  // namespace triwhirl::runtime

#endif  // TRIWHIRL_ROUTE_B_MOTOR_COMMISSIONING && backend
