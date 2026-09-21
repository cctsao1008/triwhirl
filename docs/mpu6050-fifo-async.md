# MPU6050 FIFO + asynchronous I2C bring-up

TriWhirl's MPU6050 runtime path now uses the sensor FIFO and ESP-IDF asynchronous I2C transactions instead of reading the 14-byte live register window synchronously every control generation.

## Runtime data path

```text
MPU6050 internal sampler @ 1 kHz
        |
        +-- accel XYZ + gyro XYZ -> hardware FIFO (12 bytes/sample)
        +-- DATA_RDY -> MPU_INT (physical destination currently unknown)
        |
        v
Core-0 IMU acquisition worker
        |
        +-- async FIFO_COUNT read
        +-- async FIFO_R_W burst
        |
        v
RuntimeSensorFrame -> Core 1 estimator / Balance
```

Temperature is intentionally excluded from FIFO because it is not part of the Balance state. All three gyro axes remain in FIFO so the runtime `imu map` command can still select the planar gyro axis without changing the hardware FIFO layout.

## Actual TRC-V1.0 hardware note

The vendor schematic names MPU6050 pin 12 `MPU_INT` and also shows a 2x10 `P4` header. Photographs of the actual board, despite the `TRC-V1.0` silkscreen, do not show that header or a matching populated footprint. The board instead exposes small production/test-pad arrays on the back side whose net assignments are not known from photographs alone.

Therefore the physical destination of `MPU_INT` is **unverified**. Firmware deliberately keeps:

```text
kMpu6050IntGpio = -1
kMpu6050IntRoutingVerified = false
```

No GPIO jumper should be added based only on the vendor schematic. FIFO + asynchronous I2C remain active without DRDY; the current sensor-generation request remains the scheduling authority.

See `docs/hardware-observed-trc-v1.md` for the physical-board evidence and the continuity/probing plan.

## Why not DMP

The Balance control path remains based on the repository-owned 1 kHz estimator. The DMP can be evaluated later as an observer, but the current control path avoids replacing the 1 kHz raw inertial stream with the lower-rate DMP output.

## Validation targets

After flashing, repeat `twtool diag realtime-check`. Compare the previous direct-register path against FIFO/async I2C using:

- sensor read/join failures;
- completion cadence;
- MPU transfer profile;
- Core-1 execution time and period jitter;
- sensor freshness during short Balance trials.

Hardware DRDY becomes eligible as the generation clock only after the actual `MPU_INT` PCB route is electrically verified.
