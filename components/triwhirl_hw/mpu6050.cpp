#include "triwhirl/drivers/mpu6050.hpp"

#include "esp_err.h"

namespace triwhirl {
namespace drivers {
namespace {

constexpr std::uint8_t kRegSampleRateDivider = 0x19U;
constexpr std::uint8_t kRegConfig = 0x1AU;
constexpr std::uint8_t kRegGyroConfig = 0x1BU;
constexpr std::uint8_t kRegAccelConfig = 0x1CU;
constexpr std::uint8_t kRegAccelXoutH = 0x3BU;
constexpr std::uint8_t kRegPowerManagement1 = 0x6BU;
constexpr std::uint8_t kRegWhoAmI = 0x75U;

constexpr std::uint8_t kExpectedWhoAmI = 0x68U;
constexpr int kI2cTimeoutMs = 20;
constexpr float kGravityMps2 = 9.80665F;
constexpr float kAccelLsbPerG = 8192.0F;  // +/-4 g
constexpr float kGyroLsbPerDps = 32.8F;  // +/-1000 deg/s
constexpr float kDegToRad = 0.01745329251994329577F;

std::int16_t readBigEndianI16(const std::uint8_t high,
                              const std::uint8_t low) {
  const std::uint16_t raw =
      (static_cast<std::uint16_t>(high) << 8U) |
      static_cast<std::uint16_t>(low);
  return static_cast<std::int16_t>(raw);
}

}  // namespace

bool Mpu6050::init(const i2c_master_bus_handle_t bus,
                   const std::uint8_t address) {
  if (bus == nullptr) {
    return false;
  }

  i2c_device_config_t config{};
  config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  config.device_address = address;
  config.scl_speed_hz = 400000U;
  if (i2c_master_bus_add_device(bus, &config, &device_) != ESP_OK) {
    return false;
  }

  std::uint8_t who_am_i = 0U;
  if (!readWhoAmI(&who_am_i) || who_am_i != kExpectedWhoAmI) {
    return false;
  }

  // Wake the device and use the X-axis gyro PLL as the clock source.
  if (!writeRegister(kRegPowerManagement1, 0x01U)) {
    return false;
  }

  // 1 kHz sample rate with DLPF enabled, DLPF_CFG=2.
  if (!writeRegister(kRegSampleRateDivider, 0x00U) ||
      !writeRegister(kRegConfig, 0x02U)) {
    return false;
  }

  // GYRO_FS_SEL=2 => +/-1000 deg/s. ACCEL_FS_SEL=1 => +/-4 g.
  if (!writeRegister(kRegGyroConfig, 0x10U) ||
      !writeRegister(kRegAccelConfig, 0x08U)) {
    return false;
  }

  return true;
}

bool Mpu6050::writeRegister(const std::uint8_t reg,
                            const std::uint8_t value) {
  if (device_ == nullptr) {
    return false;
  }
  const std::uint8_t data[2] = {reg, value};
  return i2c_master_transmit(device_, data, sizeof(data), kI2cTimeoutMs) == ESP_OK;
}

bool Mpu6050::readRegisters(const std::uint8_t first_register,
                            std::uint8_t* const data,
                            const std::size_t length) {
  if (device_ == nullptr || data == nullptr || length == 0U) {
    return false;
  }
  return i2c_master_transmit_receive(device_, &first_register, 1U, data, length,
                                     kI2cTimeoutMs) == ESP_OK;
}

bool Mpu6050::readWhoAmI(std::uint8_t* const who_am_i) {
  if (who_am_i == nullptr) {
    return false;
  }
  return readRegisters(kRegWhoAmI, who_am_i, 1U);
}

bool Mpu6050::readSample(Mpu6050Sample* const sample) {
  if (sample == nullptr) {
    return false;
  }

  std::uint8_t data[14]{};
  if (!readRegisters(kRegAccelXoutH, data, sizeof(data))) {
    return false;
  }

  sample->accel_raw[0] = readBigEndianI16(data[0], data[1]);
  sample->accel_raw[1] = readBigEndianI16(data[2], data[3]);
  sample->accel_raw[2] = readBigEndianI16(data[4], data[5]);
  sample->temperature_raw = readBigEndianI16(data[6], data[7]);
  sample->gyro_raw[0] = readBigEndianI16(data[8], data[9]);
  sample->gyro_raw[1] = readBigEndianI16(data[10], data[11]);
  sample->gyro_raw[2] = readBigEndianI16(data[12], data[13]);

  const float accel_scale = kGravityMps2 / kAccelLsbPerG;
  const float gyro_scale = kDegToRad / kGyroLsbPerDps;
  for (int axis = 0; axis < 3; ++axis) {
    sample->accel_mps2[axis] =
        static_cast<float>(sample->accel_raw[axis]) * accel_scale;
    sample->gyro_rad_s[axis] =
        static_cast<float>(sample->gyro_raw[axis]) * gyro_scale;
  }
  sample->temperature_c =
      static_cast<float>(sample->temperature_raw) / 340.0F + 36.53F;
  return true;
}

}  // namespace drivers
}  // namespace triwhirl
