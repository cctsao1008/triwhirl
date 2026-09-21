# Observed TRC-V1.0 production board

This note records physical observations from the user's actual TriWhirl board. It intentionally overrides unverified connector assumptions taken from the vendor schematic.

## What the photographs establish

- The PCB silkscreen is `TRC-V1.0`.
- The physical board does **not** have the schematic's 2x10 `P4` header populated or visible as a matching footprint.
- The back side has small bed-of-nails / test-pad arrays, but their nets are not identified from photographs alone.
- A 24-pin QFN device consistent with the MPU6050 package is visible on the component side near the large 1000-uF capacitor. The package orientation appears to have a pin-1 marker, but individual hidden/inner copper routing cannot be proven from the photographs.

## Consequence for MPU6050 DRDY

The vendor schematic labels MPU6050 pin 12 as `MPU_INT`, but the actual PCB routing of that net is currently unknown. Do **not** assume it reaches GPIO21 or any particular test pad.

Firmware therefore keeps hardware DRDY disabled until continuity or board-level probing establishes a real route:

```text
MPU6050 INT routing = UNKNOWN
kMpu6050IntGpio     = -1
```

FIFO + asynchronous I2C remain valid without DRDY and continue to be the active acquisition path.

## How to establish the interrupt route

Preferred order:

1. identify MPU6050 pin 12 from the package pin-1 orientation;
2. power the board off and use continuity mode from pin 12 to accessible test pads / ESP32 GPIO pads;
3. if continuity is inconclusive, observe candidate pads with an oscilloscope while DATA_RDY is enabled;
4. only after a physical net is verified, assign `kMpu6050IntGpio` and enable DRDY-driven scheduling.

Do not solder a jumper to an assumed GPIO from the vendor schematic until the actual PCB net is verified.
