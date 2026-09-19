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

constexpr float kMotorBusNominalV = 12.0F;
constexpr float kBringupPhaseAmplitudeMaxV = 1.5F;

}  // namespace board
}  // namespace triwhirl
