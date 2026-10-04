#include "runtime_simplefoc_sensor_commissioning.hpp"

#if defined(TRIWHIRL_ROUTE_B_SENSOR_COMMISSIONING) && \
    defined(TRIWHIRL_ROUTE_B_SIMPLEFOC_BACKEND)

#include <Arduino.h>

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "triwhirl/board.hpp"
#include "triwhirl/simplefoc_sensor_path.hpp"

namespace triwhirl::runtime {
namespace {

// This is the existing AS5600 bus rate used by the board runtime. It is a bus
// configuration only, not a motor-control or fuzzy tuning parameter.
constexpr std::uint32_t kSensorBusHz = 400000U;
constexpr std::uint32_t kServicePeriodMs = 1U;
constexpr std::uint32_t kTelemetryEverySamples = 50U;

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

[[noreturn]] void fatalLoop(const char* const reason) {
  std::printf("FATAL sfoc_sensor reason=%s\r\n", reason);
  std::fflush(stdout);
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

}  // namespace

[[noreturn]] void runSimpleFocSensorCommissioning() {
  // This Route-B profile uses Arduino's TwoWire implementation underneath the
  // pinned SimpleFOC MagneticSensorI2C path. Because TriWhirl supplies its own
  // ESP-IDF app_main(), initialize the Arduino core explicitly instead of
  // relying on CONFIG_AUTOSTART_ARDUINO.
  initArduino();

  simplefoc::SimpleFocSensorConfig config{};
  config.i2c_bus_index = 0;
  config.sda_gpio = board::kAs5600SdaGpio;
  config.scl_gpio = board::kAs5600SclGpio;
  config.i2c_hz = kSensorBusHz;

  if (!simplefoc::validSimpleFocSensorConfig(config)) {
    fatalLoop("invalid_config");
  }

  simplefoc::SimpleFocSensorPath sensor(config);

  std::printf(
      "TriWhirl SimpleFOC sensor-only commissioning\r\n"
      "owner=SimpleFOC,actuator=DISABLED,motor_init=ABSENT,"
      "sda=%d,scl=%d,i2c_hz=%" PRIu32 "\r\n",
      config.sda_gpio, config.scl_gpio, config.i2c_hz);
  std::fflush(stdout);

  if (!sensor.begin()) {
    fatalLoop("sensor_begin_failed");
  }

  TimingStats service_timing{};
  TimingStats period_timing{};
  std::uint64_t invalid_samples = 0U;
  std::uint32_t previous_service_start_us = 0U;
  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    const std::uint32_t service_start_us =
        static_cast<std::uint32_t>(esp_timer_get_time());
    if (previous_service_start_us != 0U) {
      period_timing.record(service_start_us - previous_service_start_us);
    }
    previous_service_start_us = service_start_us;

    const bool sample_ok = sensor.service();
    const std::uint32_t service_end_us =
        static_cast<std::uint32_t>(esp_timer_get_time());
    const std::uint32_t service_us = service_end_us - service_start_us;
    service_timing.record(service_us);

    const simplefoc::SimpleFocSensorObservation observation =
        sensor.observation();
    if (!sample_ok) {
      ++invalid_samples;
    }

    if ((service_timing.count % kTelemetryEverySamples) == 0U) {
      const std::uint32_t latest_period_us =
          period_timing.count == 0U ? 0U :
          (service_start_us - (previous_service_start_us -
                               (period_timing.count == 0U ? 0U : 0U)));
      // The latest period is reported separately below from the 1 kHz release
      // cadence statistics; min/max/mean are the useful accumulated measures.
      (void)latest_period_us;

      std::printf(
          "sfoc_sensor,t_us=%" PRIu32
          ",sample=%" PRIu64
          ",valid=%u,wire_error=%u,angle_rad=%.6f,velocity_rad_s=%.6f"
          ",service_us=%" PRIu32
          ",service_min_us=%" PRIu32
          ",service_max_us=%" PRIu32
          ",service_mean_us=%.2f"
          ",period_min_us=%" PRIu32
          ",period_max_us=%" PRIu32
          ",period_mean_us=%.2f"
          ",invalid_samples=%" PRIu64 "\r\n",
          service_end_us,
          service_timing.count,
          observation.sample_valid ? 1U : 0U,
          static_cast<unsigned>(observation.wire_error),
          static_cast<double>(observation.shaft_angle_rad),
          static_cast<double>(observation.shaft_velocity_rad_s),
          service_us,
          service_timing.visibleMinUs(),
          service_timing.max_us,
          service_timing.meanUs(),
          period_timing.visibleMinUs(),
          period_timing.max_us,
          period_timing.meanUs(),
          invalid_samples);
      std::fflush(stdout);
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(kServicePeriodMs));
  }
}

}  // namespace triwhirl::runtime

#else

namespace triwhirl::runtime {

[[noreturn]] void runSimpleFocSensorCommissioning() {
  // This translation unit is compiled into native builds as a fail-closed
  // stub. The native app_main never calls it because the selection macro is
  // absent. If an integration mistake reaches it, trap here rather than fall
  // through into any motor runtime.
  for (;;) {
  }
}

}  // namespace triwhirl::runtime

#endif
