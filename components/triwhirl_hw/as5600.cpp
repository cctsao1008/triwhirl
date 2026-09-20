#include "triwhirl/drivers/as5600.hpp"

#include "esp_err.h"

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
}  // namespace

bool As5600::init(const i2c_master_bus_handle_t bus, const std::uint8_t address) {
  if (bus == nullptr) {
    return false;
  }
  i2c_device_config_t config{};
  config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  config.device_address = address;
  config.scl_speed_hz = kI2cClockHz;
  return i2c_master_bus_add_device(bus, &config, &device_) == ESP_OK;
}

bool As5600::readRegisters(const std::uint8_t first_register,
                           std::uint8_t* const data,
                           const std::size_t length) {
  if (device_ == nullptr || data == nullptr || length == 0U) {
    return false;
  }
  return i2c_master_transmit_receive(device_, &first_register, 1U, data, length,
                                     kI2cTimeoutMs) == ESP_OK;
}

bool As5600::readRawAngle(std::uint16_t* const raw_count) {
  if (raw_count == nullptr) {
    return false;
  }
  std::uint8_t data[2]{};
  if (!readRegisters(kRawAngleHighRegister, data, sizeof(data))) {
    return false;
  }
  *raw_count = static_cast<std::uint16_t>(
      (static_cast<std::uint16_t>(data[0] & 0x0FU) << 8U) | data[1]);
  return true;
}

bool As5600::readStatus(As5600Status* const status) {
  if (status == nullptr) {
    return false;
  }
  std::uint8_t raw = 0U;
  if (!readRegisters(kStatusRegister, &raw, 1U)) {
    return false;
  }
  status->raw = raw;
  status->magnet_detected = (raw & kStatusMagnetDetected) != 0U;
  status->magnet_too_weak = (raw & kStatusMagnetTooWeak) != 0U;
  status->magnet_too_strong = (raw & kStatusMagnetTooStrong) != 0U;
  return true;
}

}  // namespace drivers
}  // namespace triwhirl
