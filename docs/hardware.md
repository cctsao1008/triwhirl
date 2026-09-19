# Hardware map

Status vocabulary:

- **schematic-derived**: directly traceable to the TRC-V1.0 schematic/netlist.
- **physically verified**: confirmed on the actual TriWhirl board used by this repository.
- **unknown**: not yet established.

## MCU and development interface

| Interface | Mapping / behavior | Status |
|---|---|---|
| MCU module | ESP-WROOM-32 / ESP32 | schematic-derived |
| USB bridge | CH340N to ESP32 UART0 path | schematic-derived; physically verified for flashing/console |
| USB connector | Type-C | schematic-derived; physically verified |
| UART console | 115200 8-N-1 on UART0 | physically verified |
| download strap | GPIO0 low | schematic-derived |
| reset | ESP32 `EN` | schematic-derived |
| board Download control | hold Download, then press Reset to enter ROM download mode | physically verified |

The actual board does not rely on automatic RTS/DTR download sequencing. The working flash sequence is:

```text
hold Download
press Reset
flash through the CH340 COM port
release Download
press Reset to run the application
```

This sequence has been used successfully with native ESP-IDF/esptool on the project hardware.

## Motor and encoder path

| Signal | ESP32 GPIO | Status |
|---|---:|---|
| `Moto_IN1` | 25 | schematic-derived |
| `Moto_IN2` | 33 | schematic-derived |
| `Moto_IN3` | 32 | schematic-derived |
| `5600_SCL` | 5 | schematic-derived; physically verified by live AS5600 reads |
| `5600_SDA` | 23 | schematic-derived; physically verified by live AS5600 reads |
| AS5600 address | `0x36` | schematic/device-defined; physically verified by communication |

The schematic uses an EG2133 three-phase gate driver. Each `Moto_INx` net is connected to both the corresponding active-high `HINx` input and active-low `LINx#` input. The board therefore exposes one complementary command per half bridge and is driven as a 3-PWM interface.

For one phase, the externally relevant driven states are:

| `Moto_INx` | EG2133 high-side output | EG2133 low-side output | Phase state |
|---:|---:|---:|---|
| 0 | off | on | phase tied to low rail |
| 1 | on | off | phase tied to high rail |

The EG2133 implements internal interlock and dead time between its high- and low-side outputs. Because `HINx` and `LINx#` are tied together on this board, firmware cannot independently request the EG2133 truth-table combinations that turn both devices of a phase off.

Consequently, commanding all three PWM duties to zero produces the low-side zero vector: all three motor phases are tied to the low rail. It produces zero commanded line-to-line voltage, but it is **not** a high-impedance motor disconnect. A spinning motor can therefore experience dynamic-braking drag in the stopped-field state.

The motor bridge is powered from the schematic's nominal 12 V boost rail. Actual bus voltage under load has not yet been measured.

## MPU6050 IMU

| Signal | ESP32 GPIO / value | Status |
|---|---:|---|
| `MPU_SCL` | GPIO18 | schematic-derived |
| `MPU_SDA` | GPIO19 | schematic-derived |
| I2C address | `0x68` | schematic-derived (`AD0` low) |
| `MPU_INT` | endpoint not relied upon by current runtime | schematic net present; exact use intentionally unspecified |

The MPU6050 uses its own ESP32 I2C controller in firmware, separate from the AS5600 bus. The runtime configures 400 kHz I2C, 1 kHz sampling, DLPF configuration 2, gyro range ±1000 °/s, and accelerometer range ±4 g.

## Other board I/O

These signals are defined in `triwhirl/board.hpp` but remain inactive until a project feature needs them.

| Signal | ESP32 GPIO | Status / note |
|---|---:|---|
| `ADC_Bat` | 34 | schematic-derived; ADC transfer function not yet characterized |
| `RGB_IN` | 4 | schematic-derived; feeds the onboard addressable RGB chain |
| `KEY1` | 13 | schematic-derived touch input |
| `KEY2` | 15 | schematic-derived touch input |
| `KEY3` | 2 | schematic-derived touch input; GPIO2 is also a boot-strapping pin, so startup level must not be disturbed |
| `GPIO0` / Download | 0 | schematic-derived; boot-strapping pin, physically verified download function |

The schematic also exposes a 20-pin header carrying several ESP32 GPIOs and serial/boot signals. The current firmware does not depend on that header for normal operation.

## Power and charging

The board is a single-cell Li-ion/LiPo design. The supplied user manual describes 4.2 V as fully charged and 3.7 V as low battery, and the schematic contains onboard charging plus boost conversion for the logic/motor rails. These values describe the battery/system hardware; TriWhirl does **not** currently use them as software undervoltage thresholds.

The Type-C connector can charge the battery and provides the CH340 USB data path. The project board must still be powered through its board power path for normal ESP32 operation during flashing, consistent with the verified bring-up procedure.

Battery safety logic remains disabled until the `ADC_Bat` divider/ADC calibration is characterized well enough to make a reliable decision.

## Items intentionally not assumed

The following are established automatically by the motor commissioning path rather than hard-coded:

- motor pole-pair count;
- motor electrical/sensor direction;
- electrical zero / sensor alignment.

The following remain measured or identified project parameters rather than assumed facts:

- safe continuous and transient `Vq`;
- hard reaction-wheel speed limit;
- actual 12 V bus behavior under load;
- battery ADC transfer function and software undervoltage threshold;
- final body-frame mapping of MPU6050 axes used by the estimator;
- the ESP32 endpoint/use of `MPU_INT` if an interrupt-driven IMU path is ever required.
