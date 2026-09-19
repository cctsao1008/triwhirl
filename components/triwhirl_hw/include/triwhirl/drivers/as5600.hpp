#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/i2c_master.h"

namespace triwhirl {
namespace drivers {

struct As5600Status {
  std::uint8_t raw = 0U;
  bool magnet_detected = false;
  bool magnet_too_weak = false;
  bool magnet_too_strong = false;
};

class As5600 {
 public:
  As5600() = default;
  bool init(i2c_master_bus_handle_t bus, std::uint8_t address = 0x36U);
  bool readRawAngle(std::uint16_t* raw_count);
  bool readStatus(As5600Status* status);

 private:
  bool readRegisters(std::uint8_t first_register,
                     std::uint8_t* data,
                     std::size_t length);
  i2c_master_dev_handle_t device_ = nullptr;
};

}  // namespace drivers
}  // namespace triwhirl
