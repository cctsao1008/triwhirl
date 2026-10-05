#include "runtime_simplefoc_sensor_commissioning.hpp"

#if defined(TRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING) && \
    defined(TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND)

#include <Arduino.h>

#if defined(TRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING_BT)
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || \
    !defined(CONFIG_BT_CLASSIC_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth sensor commissioning requires Bluedroid + Classic BT + SPP"
#endif
#endif

#include <algorithm>
#include <cinttypes>
#include <cstdio>
#include <cstdint>
#include <limits>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "triwhirl/board.hpp"
#include "triwhirl/simplefoc_sensor_path.hpp"

namespace triwhirl::runtime {
namespace {

// Existing physical AS5600 bus configuration. This is not a motor-control or
// fuzzy tuning parameter.
constexpr std::uint32_t kSensorBusHz = 400000U;
constexpr std::uint32_t kServicePeriodMs = 1U;
constexpr std::uint32_t kTelemetryPeriodMs = 50U;
constexpr UBaseType_t kSensorTaskPriority = configMAX_PRIORITIES - 3;
constexpr UBaseType_t kTelemetryTaskPriority = 2U;
constexpr BaseType_t kSensorTaskCore = 0;
constexpr BaseType_t kTelemetryTaskCore = 1;

#if defined(TRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING_BT)
constexpr char kBluetoothDeviceName[] = "TriWhirl-Sensor";
BluetoothSerial* bluetooth_telemetry = nullptr;
#endif

struct TimingStats {
  std::uint64_t count = 0U;
  std::uint64_t total_us = 0U;
  std::uint32_t min_us = std::numeric_limits<std::uint32_t>::max();
  std::uint32_t max_us = 0U;

  void record(const std::uint32_t value_us) {
    ++count;
    total_us += value_us;
    min_us = std::min(min_us, value_us);
    max_us = std::max(max_us, value_us);
  }

  double meanUs() const {
    return count == 0U ? 0.0 : static_cast<double>(total_us) /
                                  static_cast<double>(count);
  }

  std::uint32_t visibleMinUs() const {
    return count == 0U ? 0U : min_us;
  }
};

struct SensorCommissioningSnapshot {
  std::uint64_t timestamp_us = 0U;
  std::uint64_t sample_count = 0U;
  std::uint64_t invalid_samples = 0U;
  std::uint32_t service_us = 0U;
  std::uint32_t service_min_us = 0U;
  std::uint32_t service_max_us = 0U;
  double service_mean_us = 0.0;
  std::uint32_t period_us = 0U;
  std::uint32_t period_min_us = 0U;
  std::uint32_t period_max_us = 0U;
  double period_mean_us = 0.0;
  simplefoc::SimpleFocSensorObservation sensor{};
};

QueueHandle_t snapshot_queue = nullptr;

simplefoc::SimpleFocSensorConfig boardSensorConfig() {
  simplefoc::SimpleFocSensorConfig config{};
  config.i2c_bus_index = 0;
  config.sda_gpio = board::kAs5600SdaGpio;
  config.scl_gpio = board::kAs5600SclGpio;
  config.i2c_hz = kSensorBusHz;
  return config;
}

[[noreturn]] void fatalLoop(const char* const reason) {
  std::printf("FATAL sfoc_sensor reason=%s\r\n", reason);
  std::fflush(stdout);
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void emitTelemetryLine(const char* const line, const std::size_t length) {
#if defined(TRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING_BT)
  // Bluetooth SPP is a commissioning transport only. Keep all potentially
  // blocking transport work in the low-priority telemetry task and never in
  // the 1 kHz sensor service task. Skip output until a host has connected.
  if (bluetooth_telemetry == nullptr || !bluetooth_telemetry->hasClient()) {
    return;
  }
  (void)bluetooth_telemetry->write(
      reinterpret_cast<const std::uint8_t*>(line), length);
#else
  (void)std::fwrite(line, 1U, length, stdout);
  std::fflush(stdout);
#endif
}

void sensorTask(void*) {
  const simplefoc::SimpleFocSensorConfig config = boardSensorConfig();
  if (!simplefoc::validSimpleFocSensorConfig(config)) {
    fatalLoop("invalid_config");
  }

  // This is the only hardware object in the active commissioning path.
  // SimpleFocSensorPath contains TwoWire + MagneticSensorI2C only.
  simplefoc::SimpleFocSensorPath sensor(config);
  if (!sensor.begin()) {
    fatalLoop("sensor_begin_failed");
  }

  TimingStats service_timing{};
  TimingStats period_timing{};
  std::uint64_t invalid_samples = 0U;
  std::uint64_t previous_service_start_us = 0U;
  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    const std::uint64_t service_start_us =
        static_cast<std::uint64_t>(esp_timer_get_time());
    std::uint32_t period_us = 0U;
    if (previous_service_start_us != 0U) {
      const std::uint64_t delta_us = service_start_us - previous_service_start_us;
      period_us = delta_us > std::numeric_limits<std::uint32_t>::max()
                      ? std::numeric_limits<std::uint32_t>::max()
                      : static_cast<std::uint32_t>(delta_us);
      period_timing.record(period_us);
    }
    previous_service_start_us = service_start_us;

    const bool sample_ok = sensor.service();
    const std::uint64_t service_end_us =
        static_cast<std::uint64_t>(esp_timer_get_time());
    const std::uint64_t elapsed_us = service_end_us - service_start_us;
    const std::uint32_t service_us =
        elapsed_us > std::numeric_limits<std::uint32_t>::max()
            ? std::numeric_limits<std::uint32_t>::max()
            : static_cast<std::uint32_t>(elapsed_us);
    service_timing.record(service_us);

    if (!sample_ok) {
      ++invalid_samples;
    }

    SensorCommissioningSnapshot snapshot{};
    snapshot.timestamp_us = service_end_us;
    snapshot.sample_count = service_timing.count;
    snapshot.invalid_samples = invalid_samples;
    snapshot.service_us = service_us;
    snapshot.service_min_us = service_timing.visibleMinUs();
    snapshot.service_max_us = service_timing.max_us;
    snapshot.service_mean_us = service_timing.meanUs();
    snapshot.period_us = period_us;
    snapshot.period_min_us = period_timing.visibleMinUs();
    snapshot.period_max_us = period_timing.max_us;
    snapshot.period_mean_us = period_timing.meanUs();
    snapshot.sensor = sensor.observation();
    (void)xQueueOverwrite(snapshot_queue, &snapshot);

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kServicePeriodMs));
  }
}

void telemetryTask(void*) {
  TickType_t last_wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kTelemetryPeriodMs));

    SensorCommissioningSnapshot snapshot{};
    if (xQueuePeek(snapshot_queue, &snapshot, 0) != pdTRUE) {
      continue;
    }

    char line[512]{};
    const int written = std::snprintf(
        line, sizeof(line),
        "sfoc_sensor,t_us=%" PRIu64
        ",sample=%" PRIu64
        ",valid=%u,wire_error=%u,angle_rad=%.6f,velocity_rad_s=%.6f"
        ",service_us=%" PRIu32
        ",service_min_us=%" PRIu32
        ",service_max_us=%" PRIu32
        ",service_mean_us=%.2f"
        ",period_us=%" PRIu32
        ",period_min_us=%" PRIu32
        ",period_max_us=%" PRIu32
        ",period_mean_us=%.2f"
        ",invalid_samples=%" PRIu64 "\r\n",
        snapshot.timestamp_us,
        snapshot.sample_count,
        snapshot.sensor.sample_valid ? 1U : 0U,
        static_cast<unsigned>(snapshot.sensor.wire_error),
        static_cast<double>(snapshot.sensor.shaft_angle_rad),
        static_cast<double>(snapshot.sensor.shaft_velocity_rad_s),
        snapshot.service_us,
        snapshot.service_min_us,
        snapshot.service_max_us,
        snapshot.service_mean_us,
        snapshot.period_us,
        snapshot.period_min_us,
        snapshot.period_max_us,
        snapshot.period_mean_us,
        snapshot.invalid_samples);
    if (written <= 0 || static_cast<std::size_t>(written) >= sizeof(line)) {
      continue;
    }
    emitTelemetryLine(line, static_cast<std::size_t>(written));
  }
}

}  // namespace

[[noreturn]] void runSimpleFocSensorCommissioning() {
  // TriWhirl supplies app_main in the dual-framework build, so initialize the
  // Arduino HAL explicitly before constructing the pinned TwoWire-based
  // SimpleFOC sensor path. CONFIG_AUTOSTART_ARDUINO is not relied upon here.
  initArduino();

#if defined(TRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING_BT)
  // Construct Bluetooth only after the Arduino HAL is live. The object remains
  // valid after this entry task deletes itself because it has static storage.
  static BluetoothSerial bluetooth;
  const BTStatus bluetooth_status = bluetooth.begin(kBluetoothDeviceName);
  if (!bluetooth_status) {
    std::printf("Bluetooth SPP init failed status=%s\r\n",
                bluetooth_status.toString());
    std::fflush(stdout);
    fatalLoop("bluetooth_spp_begin_failed");
  }
  bluetooth_telemetry = &bluetooth;
  const String bluetooth_address = bluetooth.getAddress().toString();
  std::printf(
      "TriWhirl SimpleFOC sensor-only commissioning\r\n"
      "transport=bluetooth_spp,name=%s,address=%s,"
      "owner=SimpleFOC,actuator=DISABLED,motor_init=ABSENT\r\n",
      kBluetoothDeviceName, bluetooth_address.c_str());
#else
  std::printf(
      "TriWhirl SimpleFOC sensor-only commissioning\r\n"
      "transport=uart,owner=SimpleFOC,actuator=DISABLED,motor_init=ABSENT\r\n");
#endif

  const simplefoc::SimpleFocSensorConfig config = boardSensorConfig();
  std::printf(
      "sda=%d,scl=%d,i2c_hz=%" PRIu32 ",service_hz=1000,telemetry_hz=20\r\n",
      config.sda_gpio, config.scl_gpio, config.i2c_hz);
  std::fflush(stdout);

  snapshot_queue = xQueueCreate(1U, sizeof(SensorCommissioningSnapshot));
  if (snapshot_queue == nullptr) {
    fatalLoop("snapshot_queue_create_failed");
  }

  if (xTaskCreatePinnedToCore(telemetryTask, "sfoc_sensor_telemetry", 4096,
                              nullptr, kTelemetryTaskPriority, nullptr,
                              kTelemetryTaskCore) != pdPASS) {
    fatalLoop("telemetry_task_create_failed");
  }

  if (xTaskCreatePinnedToCore(sensorTask, "sfoc_sensor", 4096, nullptr,
                              kSensorTaskPriority, nullptr,
                              kSensorTaskCore) != pdPASS) {
    fatalLoop("sensor_task_create_failed");
  }

  vTaskDelete(nullptr);
  for (;;) {
  }
}

}  // namespace triwhirl::runtime

#endif  // TRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING && backend
