#pragma once

#include <cstdint>

namespace triwhirl {
namespace board {

// TRC-V1.0 schematic-derived assignments. These are not yet physically verified.
constexpr int kMotorIn1Gpio = 25;
constexpr int kMotorIn2Gpio = 33;
constexpr int kMotorIn3Gpio = 32;

constexpr int kAs5600SclGpio = 5;
constexpr int kAs5600SdaGpio = 23;
constexpr std::uint8_t kAs5600I2cAddress = 0x36;

// Motor bridge is fed from the schematic's nominal 12 V rail.
constexpr float kMotorBusNominalV = 12.0F;

// Conservative bring-up ceiling. This is a software test limit, not a measured
// safe operating limit and must be revisited after hardware characterization.
constexpr float kBringupPhaseAmplitudeMaxV = 1.5F;

}  // namespace board
}  // namespace triwhirl
