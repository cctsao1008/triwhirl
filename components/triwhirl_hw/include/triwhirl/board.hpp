#pragma once

#include <cstdint>

namespace triwhirl {
namespace board {

// Schematic net ownership.
constexpr int kMotorIn1Gpio = 25;
constexpr int kMotorIn2Gpio = 33;
constexpr int kMotorIn3Gpio = 32;

// Golden TRC-V1.1 8 V firmware instantiates BLDCDriver3PWM(33, 25, 32).
// Keep phase order explicit instead of changing the schematic net names above.
constexpr int kMotorPhaseAGpio = kMotorIn2Gpio;  // GPIO33
constexpr int kMotorPhaseBGpio = kMotorIn1Gpio;  // GPIO25
constexpr int kMotorPhaseCGpio = kMotorIn3Gpio;  // GPIO32

constexpr int kAs5600SclGpio = 5;
constexpr int kAs5600SdaGpio = 23;
constexpr std::uint8_t kAs5600I2cAddress = 0x36U;

constexpr int kMpu6050SclGpio = 18;
constexpr int kMpu6050SdaGpio = 19;
constexpr std::uint8_t kMpu6050I2cAddress = 0x68U;

// TRC-V1.0 labels MPU6050 pin 12 as MPU_INT, but the schematic does not connect
// that net to an ESP32 GPIO. GPIO21 is a separate P4 net, and the physical-unit
// passive probe observed zero DATA_RDY edges while the MPU was running at 1 kHz.
// Do not keep probing IO21. When an actual MPU_INT -> ESP32 route is established
// (existing PCB trace or an intentional jumper), set kMpu6050IntGpio to that pin
// and kMpu6050IntRoutingVerified=true; runtime acquisition will then become
// DATA_RDY-IRQ-owned automatically.
constexpr int kMpu6050IntGpio = -1;
constexpr int kMpu6050IntProbeGpio = -1;
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

// Golden TRC-V1.1 8 V motor firmware uses BLDCMotor(7), an 8.3 V driver supply,
// SimpleFOC's 3 V sensor-alignment field, and a 4 V motor voltage limit.
constexpr int kMotorPolePairs = 7;
constexpr float kMotorBusNominalV = 8.3F;
constexpr float kMotorSensorAlignVoltageV = 3.0F;

// Keep the conservative realtime bring-up/control envelope separate from the
// short-lived calibration alignment field. Widening calibration must not widen
// Balance/Swing Vq authority.
constexpr float kBringupPhaseAmplitudeMaxV = 1.5F;
constexpr float kMotorCalibrationVectorLimitV = 3.0F;

}  // namespace board
}  // namespace triwhirl
