#pragma once

#include <cstdint>

namespace triwhirl {
namespace board {

// Physical schematic nets on TRC-V1.x.
constexpr int kMotorNetIn1Gpio = 25;
constexpr int kMotorNetIn2Gpio = 33;
constexpr int kMotorNetIn3Gpio = 32;

// Runtime bridge phase order. The golden TRC-V1.1 8 V firmware instantiates
// BLDCDriver3PWM(33, 25, 32), so preserve that proven A/B/C ordering while
// keeping the physical Moto_IN1/2/3 net mapping explicit above.
constexpr int kMotorIn1Gpio = kMotorNetIn2Gpio;  // phase A, GPIO33
constexpr int kMotorIn2Gpio = kMotorNetIn1Gpio;  // phase B, GPIO25
constexpr int kMotorIn3Gpio = kMotorNetIn3Gpio;  // phase C, GPIO32
constexpr int kMotorPhaseAGpio = kMotorIn1Gpio;
constexpr int kMotorPhaseBGpio = kMotorIn2Gpio;
constexpr int kMotorPhaseCGpio = kMotorIn3Gpio;

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
// SimpleFOC's 3 V sensor-alignment field, and a 4 V motor voltage limit. A 3 V
// TriWhirl envelope is still conservative relative to that proven 4 V limit.
constexpr int kMotorPolePairs = 7;
constexpr float kMotorBusNominalV = 8.3F;
constexpr float kMotorSensorAlignVoltageV = 3.0F;
constexpr float kBringupPhaseAmplitudeMaxV = 3.0F;
constexpr float kMotorCalibrationVectorLimitV = 3.0F;

}  // namespace board
}  // namespace triwhirl
