#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

namespace triwhirl {
namespace drivers {

struct Mpu6050Sample {
  std::int16_t accel_raw[3]{};
  std::int16_t temperature_raw = 0;
  std::int16_t gyro_raw[3]{};
  float accel_mps2[3]{};
  float temperature_c = 0.0F;
  float gyro_rad_s[3]{};
};

struct Mpu6050TimingStats {
  std::uint64_t sample_reads = 0U;
  std::uint64_t transfer_total_us = 0U;
  std::uint32_t transfer_min_us = 0U;
  std::uint32_t transfer_max_us = 0U;
  std::uint64_t decode_total_us = 0U;
  std::uint32_t decode_min_us = 0U;
  std::uint32_t decode_max_us = 0U;
};

class Mpu6050 {
 public:
  Mpu6050() = default;

  bool init(i2c_master_bus_handle_t bus, std::uint8_t address = 0x68U);

  // Identity is probed during init and cached. After successful init this is a
  // pure cached read, so diagnostic status commands never issue live I2C.
  bool readWhoAmI(std::uint8_t* who_am_i);
  bool readSample(Mpu6050Sample* sample);

  // Timing-profile control can be called from Core 1 while readSample() runs on
  // the Core-0 acquisition worker. Keep this diagnostic state cross-core safe.
  void setTimingProfileEnabled(bool enabled);
  void resetTimingProfile();
  Mpu6050TimingStats timingProfile() const;

 private:
  bool writeRegister(std::uint8_t reg, std::uint8_t value);
  bool readRegisters(std::uint8_t first_register,
                     std::uint8_t* data,
                     std::size_t length);
  void recordSampleTiming(std::uint32_t transfer_us, std::uint32_t decode_us);

  i2c_master_dev_handle_t device_ = nullptr;
  std::uint8_t who_am_i_ = 0U;
  bool who_am_i_valid_ = false;
  std::atomic<bool> timing_profile_enabled_{false};
  mutable portMUX_TYPE timing_mux_ = portMUX_INITIALIZER_UNLOCKED;
  Mpu6050TimingStats timing_stats_{};
};

}  // namespace drivers
}  // namespace triwhirl
