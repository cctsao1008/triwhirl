#include "triwhirl/drivers/mpu6050.hpp"

#include <algorithm>

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
constexpr std::uint32_t kPowerOnSettleMs = 100U;
constexpr std::uint32_t kWakeSettleMs = 30U;
constexpr std::uint32_t kRetrySettleMs = 30U;
constexpr unsigned kInitAttempts = 3U;
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

  // The MPU-60X0 may not accept register traffic immediately after power-on.
  // app_main can run quickly after reset, so give the device a deterministic
  // startup window before probing it.
  vTaskDelay(pdMS_TO_TICKS(kPowerOnSettleMs));

  const std::uint8_t alternate = address == 0x68U ? 0x69U : 0x68U;
  const std::uint8_t candidates[2] = {address, alternate};

  // A firmware/flash reset resets the ESP32 I2C controller without necessarily
  // power-cycling the external MPU6050.  If a reset interrupts an I2C transfer,
  // the first probe/register sequence can fail even though a full power cycle
  // immediately restores the sensor.  Retry locally and reset the master bus
  // between attempts so a warm reset does not permanently disable IMU support
  // for the rest of that boot.
  for (unsigned attempt = 0U; attempt < kInitAttempts; ++attempt) {
    if (attempt > 0U) {
      if (device_ != nullptr) {
        i2c_master_bus_rm_device(device_);
        device_ = nullptr;
      }
      i2c_master_bus_reset(bus);
      vTaskDelay(pdMS_TO_TICKS(kRetrySettleMs));
    }

    for (const std::uint8_t candidate : candidates) {
      if (i2c_master_probe(bus, candidate, kI2cTimeoutMs) != ESP_OK) {
        continue;
      }

      i2c_device_config_t config{};
      config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
      config.device_address = candidate;
      config.scl_speed_hz = 400000U;
      if (i2c_master_bus_add_device(bus, &config, &device_) != ESP_OK) {
        device_ = nullptr;
        continue;
      }

      auto discard_device = [this]() {
        if (device_ != nullptr) {
          i2c_master_bus_rm_device(device_);
          device_ = nullptr;
        }
      };

      std::uint8_t who_am_i = 0U;
      if (!readWhoAmI(&who_am_i) || who_am_i != kExpectedWhoAmI) {
        discard_device();
        continue;
      }

      // Wake the device and use the X-axis gyro PLL as the clock source.
      if (!writeRegister(kRegPowerManagement1, 0x01U)) {
        discard_device();
        continue;
      }

      // The gyro needs a short settling interval after wake before its output is
      // used for bias calibration and attitude estimation.
      vTaskDelay(pdMS_TO_TICKS(kWakeSettleMs));

      // 1 kHz sample rate with DLPF enabled, DLPF_CFG=2.
      if (!writeRegister(kRegSampleRateDivider, 0x00U) ||
          !writeRegister(kRegConfig, 0x02U) ||
          // GYRO_FS_SEL=2 => +/-1000 deg/s.
          !writeRegister(kRegGyroConfig, 0x10U) ||
          // ACCEL_FS_SEL=1 => +/-4 g.
          !writeRegister(kRegAccelConfig, 0x08U)) {
        discard_device();
        continue;
      }

      return true;
    }
  }

  return false;
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
  const std::int64_t transfer_begin_us =
      timing_profile_enabled_ ? esp_timer_get_time() : 0;
  if (!readRegisters(kRegAccelXoutH, data, sizeof(data))) {
    return false;
  }
  const std::int64_t transfer_end_us =
      timing_profile_enabled_ ? esp_timer_get_time() : 0;

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

  if (timing_profile_enabled_) {
    const std::int64_t decode_end_us = esp_timer_get_time();
    recordSampleTiming(
        static_cast<std::uint32_t>(transfer_end_us - transfer_begin_us),
        static_cast<std::uint32_t>(decode_end_us - transfer_end_us));
  }
  return true;
}

void Mpu6050::setTimingProfileEnabled(const bool enabled) {
  timing_profile_enabled_ = enabled;
}

void Mpu6050::resetTimingProfile() {
  timing_stats_ = {};
}

Mpu6050TimingStats Mpu6050::timingProfile() const {
  return timing_stats_;
}

void Mpu6050::recordSampleTiming(const std::uint32_t transfer_us,
                                 const std::uint32_t decode_us) {
  ++timing_stats_.sample_reads;
  timing_stats_.transfer_total_us += transfer_us;
  timing_stats_.decode_total_us += decode_us;
  if (timing_stats_.sample_reads == 1U) {
    timing_stats_.transfer_min_us = transfer_us;
    timing_stats_.transfer_max_us = transfer_us;
    timing_stats_.decode_min_us = decode_us;
    timing_stats_.decode_max_us = decode_us;
    return;
  }
  timing_stats_.transfer_min_us =
      std::min(timing_stats_.transfer_min_us, transfer_us);
  timing_stats_.transfer_max_us =
      std::max(timing_stats_.transfer_max_us, transfer_us);
  timing_stats_.decode_min_us =
      std::min(timing_stats_.decode_min_us, decode_us);
  timing_stats_.decode_max_us =
      std::max(timing_stats_.decode_max_us, decode_us);
}

}  // namespace drivers
}  // namespace triwhirl
