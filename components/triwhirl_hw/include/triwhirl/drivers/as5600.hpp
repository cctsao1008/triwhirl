#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace triwhirl {
namespace drivers {

struct As5600Status {
  std::uint8_t raw = 0U;
  bool magnet_detected = false;
  bool magnet_too_weak = false;
  bool magnet_too_strong = false;
  std::uint8_t agc = 0U;
  std::uint16_t magnitude = 0U;
};

struct As5600TimingStats {
  std::uint64_t raw_reads = 0U;
  std::uint64_t raw_total_us = 0U;
  std::uint32_t raw_min_us = 0U;
  std::uint32_t raw_max_us = 0U;
  std::uint64_t status_reads = 0U;
  std::uint64_t status_total_us = 0U;
  std::uint32_t status_min_us = 0U;
  std::uint32_t status_max_us = 0U;
};

class As5600 {
 public:
  As5600() = default;
  bool init(i2c_master_bus_handle_t bus, std::uint8_t address = 0x36U);
  bool readRawAngle(std::uint16_t* raw_count);
  bool readStatus(As5600Status* status);

  void setTimingProfileEnabled(bool enabled);
  void resetTimingProfile();
  As5600TimingStats timingProfile() const;

 private:
  bool selectRegister(std::uint8_t reg);
  bool readRegisters(std::uint8_t first_register,
                     std::uint8_t* data,
                     std::size_t length);
  void recordRawTiming(std::uint32_t elapsed_us);
  void recordStatusTiming(std::uint32_t elapsed_us);

  i2c_master_dev_handle_t device_ = nullptr;
  SemaphoreHandle_t mutex_ = nullptr;
  bool raw_angle_pointer_valid_ = false;
  bool timing_profile_enabled_ = false;
  As5600TimingStats timing_stats_{};
};

}  // namespace drivers
}  // namespace triwhirl
