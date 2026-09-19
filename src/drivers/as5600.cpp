#include "triwhirl/drivers/as5600.hpp"

#include <Wire.h>

namespace triwhirl {
namespace drivers {
namespace {

constexpr std::uint8_t kStatusRegister = 0x0BU;
constexpr std::uint8_t kRawAngleHighRegister = 0x0CU;
constexpr std::uint8_t kStatusMagnetDetected = 1U << 5;
constexpr std::uint8_t kStatusMagnetTooWeak = 1U << 4;
constexpr std::uint8_t kStatusMagnetTooStrong = 1U << 3;

}  // namespace

As5600::As5600(TwoWire& wire, const std::uint8_t address)
    : wire_(wire), address_(address) {}

bool As5600::readRegisters(const std::uint8_t first_register,
                           std::uint8_t* const data,
                           const std::size_t length) {
  if (data == nullptr || length == 0U || length > 255U) {
    return false;
  }

  wire_.beginTransmission(address_);
  wire_.write(first_register);
  if (wire_.endTransmission(false) != 0U) {
    return false;
  }

  const std::uint8_t requested = static_cast<std::uint8_t>(length);
  const std::size_t received = wire_.requestFrom(address_, requested);
  if (received != length) {
    while (wire_.available() > 0) {
      static_cast<void>(wire_.read());
    }
    return false;
  }

  for (std::size_t i = 0; i < length; ++i) {
    if (wire_.available() <= 0) {
      return false;
    }
    data[i] = static_cast<std::uint8_t>(wire_.read());
  }
  return true;
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
