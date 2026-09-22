#include "triwhirl/drivers/as5600.hpp"

#include <algorithm>

#include "esp_err.h"
#include "esp_timer.h"

namespace triwhirl {
namespace drivers {
namespace {
constexpr std::uint8_t kStatusRegister = 0x0BU;
constexpr std::uint8_t kRawAngleHighRegister = 0x0CU;
constexpr std::uint8_t kAgcRegister = 0x1AU;
constexpr std::uint8_t kStatusMagnetDetected = 1U << 5;
constexpr std::uint8_t kStatusMagnetTooWeak = 1U << 4;
constexpr std::uint8_t kStatusMagnetTooStrong = 1U << 3;
constexpr int kI2cTimeoutMs = 20;
// The sensor worker and live field diagnostics must never inherit a long bus
// timeout. Normal transfers are a few hundred microseconds on this board; two
// milliseconds leaves margin while bounding a stuck transaction tightly.
constexpr int kRuntimeI2cTimeoutMs = 2;
// Match the vendor TRC-V1.1 golden firmware. AS5600 supports Fast-mode Plus,
// but the proven balancing image runs this physical encoder bus at 400 kHz.
// Keep the cached RAW_ANGLE receive-only fast path, while using the vendor bus
// rate for better margin once 25 kHz motor PWM is active.
constexpr std::uint32_t kI2cClockHz = 400000U;

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
  return i2c_master_transmit(device_, &reg, 1U, kRuntimeI2cTimeoutMs) == ESP_OK;
}

bool As5600::readRegisters(const std::uint8_t first_register,
                           std::uint8_t* const data,
                           const std::size_t length) {
  if (device_ == nullptr || data == nullptr || length == 0U) {
    return false;
  }
  // Any addressed register transaction changes the AS5600 internal address
  // pointer. Invalidate the RAW ANGLE fast-path so its next sample reseeds 0x0C.
  raw_angle_pointer_valid_ = false;
  return i2c_master_transmit_receive(device_, &first_register, 1U, data, length,
                                     kRuntimeI2cTimeoutMs) == ESP_OK;
}

bool As5600::readRawAngle(std::uint16_t* const raw_count) {
  if (raw_count == nullptr || device_ == nullptr || mutex_ == nullptr) {
    return false;
  }
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(kRuntimeI2cTimeoutMs)) != pdTRUE) {
    return false;
  }

  const bool profile = timing_profile_enabled_;
  const std::int64_t begin_us = profile ? esp_timer_get_time() : 0;

  bool ok = true;
  std::uint8_t data[2]{};
  if (!raw_angle_pointer_valid_) {
    const std::uint8_t first_register = kRawAngleHighRegister;
    ok = i2c_master_transmit_receive(
             device_, &first_register, 1U, data, sizeof(data),
             kRuntimeI2cTimeoutMs) == ESP_OK;
    if (ok) {
      raw_angle_pointer_valid_ = true;
    }
  } else {
    ok = i2c_master_receive(device_, data, sizeof(data),
                            kRuntimeI2cTimeoutMs) == ESP_OK;
    if (!ok) {
      raw_angle_pointer_valid_ = false;
    }
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
  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(kRuntimeI2cTimeoutMs)) != pdTRUE) {
    return false;
  }

  const bool profile = timing_profile_enabled_;
  const std::int64_t begin_us = profile ? esp_timer_get_time() : 0;
  std::uint8_t raw = 0U;
  std::uint8_t field[3]{};
  const bool status_ok = readRegisters(kStatusRegister, &raw, 1U);
  const bool field_ok = status_ok &&
                        readRegisters(kAgcRegister, field, sizeof(field));
  const bool ok = status_ok && field_ok;
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
  status->agc = field[0];
  status->magnitude = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(field[1] & 0x0FU) << 8U) | field[2]);
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
