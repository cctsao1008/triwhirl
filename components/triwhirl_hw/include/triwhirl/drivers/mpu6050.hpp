#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

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

  // Runtime samples come from the MPU hardware FIFO. FIFO_COUNT and FIFO_R_W
  // transfers are submitted through ESP-IDF's asynchronous I2C master path;
  // this caller sleeps on an ISR-signalled semaphore instead of occupying CPU
  // while SCL/SDA shift the bytes.
  bool readSample(Mpu6050Sample* sample);

  bool fifoEnabled() const { return fifo_enabled_; }
  bool asyncI2cEnabled() const { return async_i2c_enabled_; }

  // Timing-profile control can be called from Core 1 while readSample() runs on
  // the Core-0 acquisition worker. Keep this diagnostic state cross-core safe.
  void setTimingProfileEnabled(bool enabled);
  void resetTimingProfile();
  Mpu6050TimingStats timingProfile() const;

 private:
  static constexpr std::size_t kFifoSampleBytes = 12U;
  static constexpr std::size_t kFifoReadBufferBytes = 48U;

  static bool asyncTransactionDone(i2c_master_dev_handle_t i2c_dev,
                                   const i2c_master_event_data_t* event,
                                   void* context);

  bool writeRegister(std::uint8_t reg, std::uint8_t value);
  bool readRegisters(std::uint8_t first_register,
                     std::uint8_t* data,
                     std::size_t length);
  bool configureRuntimeFifo();
  bool asyncRead(std::uint8_t first_register,
                 std::uint8_t* data,
                 std::size_t length,
                 TickType_t timeout_ticks);
  bool decodeFifoSample(const std::uint8_t* data,
                        std::size_t length,
                        Mpu6050Sample* sample) const;
  void recordSampleTiming(std::uint32_t transfer_us, std::uint32_t decode_us);

  i2c_master_dev_handle_t device_ = nullptr;
  std::uint8_t who_am_i_ = 0U;
  bool who_am_i_valid_ = false;
  bool fifo_enabled_ = false;
  bool async_i2c_enabled_ = false;

  SemaphoreHandle_t async_done_ = nullptr;
  std::atomic<bool> async_in_flight_{false};
  std::atomic<std::uint8_t> async_event_{0U};
  std::uint8_t async_register_ = 0U;
  std::uint8_t fifo_count_data_[2]{};
  std::uint8_t fifo_data_[kFifoReadBufferBytes]{};

  std::atomic<bool> timing_profile_enabled_{false};
  mutable portMUX_TYPE timing_mux_ = portMUX_INITIALIZER_UNLOCKED;
  Mpu6050TimingStats timing_stats_{};
};

}  // namespace drivers
}  // namespace triwhirl
