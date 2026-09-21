#pragma once

#include <cstdint>

namespace triwhirl {
namespace board {

constexpr int kMotorIn1Gpio = 25;
constexpr int kMotorIn2Gpio = 33;
constexpr int kMotorIn3Gpio = 32;

constexpr int kAs5600SclGpio = 5;
constexpr int kAs5600SdaGpio = 23;
constexpr std::uint8_t kAs5600I2cAddress = 0x36U;

constexpr int kMpu6050SclGpio = 18;
constexpr int kMpu6050SdaGpio = 19;
constexpr std::uint8_t kMpu6050I2cAddress = 0x68U;

// TRC-V1.0 labels MPU6050 pin 12 as MPU_INT but does not route that net to the
// ESP32. GPIO21 is otherwise exposed on P4 and is the chosen bring-up jumper
// target. Firmware keeps a pull-down on this input, so an unmodified board can
// continue using the FIFO path without spurious edges. Populate a jumper from
// MPU_INT to IO21 to activate hardware DRDY observability.
constexpr int kMpu6050IntGpio = 21;
constexpr bool kMpu6050IntRequiresJumper = true;

// Additional schematic-defined board interfaces. These are named here so a
// future driver does not have to rediscover the PCB mapping. They are not
// enabled by the current control runtime unless explicitly used.
constexpr int kBatteryAdcGpio = 34;
constexpr int kRgbDataGpio = 4;
constexpr int kKey1Gpio = 13;
constexpr int kKey2Gpio = 15;
constexpr int kKey3Gpio = 2;
constexpr int kDownloadGpio = 0;

constexpr float kMotorBusNominalV = 12.0F;
constexpr float kBringupPhaseAmplitudeMaxV = 1.5F;

}  // namespace board
}  // namespace triwhirl
