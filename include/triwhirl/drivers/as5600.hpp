#pragma once

#include <cstddef>
#include <cstdint>

class TwoWire;

namespace triwhirl {
namespace drivers {

struct As5600Status {
  std::uint8_t raw = 0;
  bool magnet_detected = false;
  bool magnet_too_weak = false;
  bool magnet_too_strong = false;
};

class As5600 {
 public:
  explicit As5600(TwoWire& wire, std::uint8_t address = 0x36U);

  bool readRawAngle(std::uint16_t* raw_count);
  bool readStatus(As5600Status* status);

 private:
  bool readRegisters(std::uint8_t first_register,
                     std::uint8_t* data,
                     std::size_t length);

  TwoWire& wire_;
  std::uint8_t address_;
};

}  // namespace drivers
}  // namespace triwhirl
