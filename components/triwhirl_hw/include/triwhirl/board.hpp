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

// The vendor TRC-V1.0 schematic shows MPU_INT and a P4 header, but photographs
// of the actual TRC-V1.0 production board do not show that header and do not
// establish which ESP32 GPIO, if any, receives MPU6050 pin 12. The vendor V1.1
// source also never consumes MPU_INT; GPIO21 is unused there. Treat IO21 only as
// a non-authoritative passive probe hypothesis until a ~1 kHz DATA_RDY waveform
// is observed or continuity confirms the route. Balance never depends on this
// probe while routing remains unverified.
constexpr int kMpu6050IntGpio = -1;
constexpr int kMpu6050IntProbeGpio = 21;
constexpr bool kMpu6050IntRoutingVerified = false;

// Additional schematic-defined board interfaces. These are named here so a
// future driver does not have to rediscover the PCB mapping. They are not
// enabled by the current control runtime unless explicitly used and verified
// on the physical board revision.
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
