#pragma once

#include <cstdint>

namespace triwhirl::board {

// TRC-V1.0 schematic-derived assignments. These are not yet physically verified.
inline constexpr int kMotorIn1Gpio = 25;
inline constexpr int kMotorIn2Gpio = 33;
inline constexpr int kMotorIn3Gpio = 32;

inline constexpr int kAs5600SclGpio = 5;
inline constexpr int kAs5600SdaGpio = 23;
inline constexpr std::uint8_t kAs5600I2cAddress = 0x36;

// Motor bridge is fed from the schematic's nominal 12 V rail.
inline constexpr float kMotorBusNominalV = 12.0F;

// Conservative bring-up ceiling. This is a software test limit, not a measured
// safe operating limit and must be revisited after hardware characterization.
inline constexpr float kBringupPhaseAmplitudeMaxV = 1.5F;

}  // namespace triwhirl::board
