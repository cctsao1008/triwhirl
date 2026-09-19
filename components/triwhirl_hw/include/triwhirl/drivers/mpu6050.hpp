#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"

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

class Mpu6050 {
 public:
  Mpu6050() = default;

  bool init(i2c_master_bus_handle_t bus, std::uint8_t address = 0x68U);
  bool readWhoAmI(std::uint8_t* who_am_i);
  bool readSample(Mpu6050Sample* sample);

 private:
  bool writeRegister(std::uint8_t reg, std::uint8_t value);
  bool readRegisters(std::uint8_t first_register,
                     std::uint8_t* data,
                     std::size_t length);

  i2c_master_dev_handle_t device_ = nullptr;
};

}  // namespace drivers
}  // namespace triwhirl
