#include "triwhirl/drivers/as5600.hpp"

#include <algorithm>

#include "esp_err.h"
#include "esp_timer.h"

namespace triwhirl {
namespace drivers {
namespace {
constexpr std::uint8_t kStatusRegister = 0x0BU;
constexpr std::uint8_t kRawAngleHighRegister = 0x0CU;
constexpr std::uint8_t kStatusMagnetDetected = 1U << 5;
constexpr std::uint8_t kStatusMagnetTooWeak = 1U << 4;
constexpr std::uint8_t kStatusMagnetTooStrong = 1U << 3;
constexpr int kI2cTimeoutMs = 20;
constexpr std::uint32_t kI2cClockHz = 1000000U;  // AS5600 Fast-mode Plus max.

void recordTiming(std::uint64_t* const count,
                  std::uint64_t* const total_us,
                  std::uint32_t* const min_us,
                  std::uint32_t* const max_us,
                  const std::uint32_t elapsed_us) {
  if (count == nullptr || total_us == nullptr || min_us == nullptr ||
      max_us == nullptr) {
    return;
  }
  ++(*count);
  *total_us += elapsed_us;
  if (*count == 1U) {
    *min_us = elapsed_us;
    *max_us = elapsed_us;
    return;
  }
  *min_us = std::min(*min_us, elapsed_us);
  *max_us = std::max(*max_us, elapsed_us);
}
}  // namespace

bool As5600::init(const i2c_master_bus_handle_t bus, const std::uint8_t address) {
  if (bus == nullptr) {
    return false;
  }
  if (mutex_ == nullptr) {
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr) {
      return false;
    }
  }
  i2c_device_config_t config{};
  config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  config.device_address = address;
  config.scl_speed_hz = kI2cClockHz;
  raw_angle_pointer_valid_ = false;
  return i2c_master_bus_add_device(bus, &config, &device_) == ESP_OK;
}

bool As5600::selectRegister(const std::uint8_t reg) {
  if (device_ == nullptr) {
    return false;
  }
  raw_angle_pointer_valid_ = false;
  return i2c_master_transmit(device_, &reg, 1U, kI2cTimeoutMs) == ESP_OK;
}

bool As5600::readRegisters(const std::uint8_t first_register,
                           std::uint8_t* const data,
                           const std::size_t length) {
  if (device_ == nullptr || data == nullptr || length == 0U) {
    return false;
  }
  // Any addressed register transaction changes the AS5600 internal address
  // pointer. The next fast RAW ANGLE read therefore has to seed 0x0C again.
  raw_angle_pointer_valid_ = false;
  return i2c_master_transmit_receive(device_, &first_register, 1U, data, length,
                                     kI2cTimeoutMs) == ESP_OK;
}

bool As5600::readRawAngle(std::uint16_t* const raw_count) {
  if (raw_count == nullptr || device_ == nullptr || mutex_ == nullptr) {
    return false;
  }
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(kI2cTimeoutMs)) != pdTRUE) {
    return false;
  }

  const bool profile = timing_profile_enabled_;
  const std::int64_t begin_us = profile ? esp_timer_get_time() : 0;

  bool ok = true;
  if (!raw_angle_pointer_valid_) {
    ok = selectRegister(kRawAngleHighRegister);
    if (ok) {
      raw_angle_pointer_valid_ = true;
    }
  }

  std::uint8_t data[2]{};
  if (ok && i2c_master_receive(device_, data, sizeof(data), kI2cTimeoutMs) != ESP_OK) {
    raw_angle_pointer_valid_ = false;
    ok = false;
  }

  if (ok) {
    *raw_count = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[0] & 0x0FU) << 8U) | data[1]);
  }
  if (profile) {
    recordRawTiming(static_cast<std::uint32_t>(esp_timer_get_time() - begin_us));
  }
  xSemaphoreGive(mutex_);
  return ok;
}

bool As5600::readStatus(As5600Status* const status) {
  if (status == nullptr || mutex_ == nullptr) {
    return false;
  }
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(kI2cTimeoutMs)) != pdTRUE) {
    return false;
  }

  const bool profile = timing_profile_enabled_;
  const std::int64_t begin_us = profile ? esp_timer_get_time() : 0;
  std::uint8_t raw = 0U;
  const bool ok = readRegisters(kStatusRegister, &raw, 1U);
  if (profile) {
    recordStatusTiming(
        static_cast<std::uint32_t>(esp_timer_get_time() - begin_us));
  }
  xSemaphoreGive(mutex_);
  if (!ok) {
    return false;
  }
  status->raw = raw;
  status->magnet_detected = (raw & kStatusMagnetDetected) != 0U;
  status->magnet_too_weak = (raw & kStatusMagnetTooWeak) != 0U;
  status->magnet_too_strong = (raw & kStatusMagnetTooStrong) != 0U;
  return true;
}

void As5600::setTimingProfileEnabled(const bool enabled) {
  if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(kI2cTimeoutMs)) != pdTRUE) {
    return;
  }
  timing_profile_enabled_ = enabled;
  xSemaphoreGive(mutex_);
}

void As5600::resetTimingProfile() {
  if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(kI2cTimeoutMs)) != pdTRUE) {
    return;
  }
  timing_stats_ = {};
  xSemaphoreGive(mutex_);
}

As5600TimingStats As5600::timingProfile() const {
  if (mutex_ == nullptr || xSemaphoreTake(mutex_, pdMS_TO_TICKS(kI2cTimeoutMs)) != pdTRUE) {
    return {};
  }
  const As5600TimingStats result = timing_stats_;
  xSemaphoreGive(mutex_);
  return result;
}

void As5600::recordRawTiming(const std::uint32_t elapsed_us) {
  recordTiming(&timing_stats_.raw_reads, &timing_stats_.raw_total_us,
               &timing_stats_.raw_min_us, &timing_stats_.raw_max_us,
               elapsed_us);
}

void As5600::recordStatusTiming(const std::uint32_t elapsed_us) {
  recordTiming(&timing_stats_.status_reads, &timing_stats_.status_total_us,
               &timing_stats_.status_min_us, &timing_stats_.status_max_us,
               elapsed_us);
}

}  // namespace drivers
}  // namespace triwhirl
