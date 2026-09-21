# MPU6050 FIFO + asynchronous I2C bring-up

TriWhirl's MPU6050 runtime path now uses the sensor FIFO and ESP-IDF asynchronous I2C transactions instead of reading the 14-byte live register window synchronously every control generation.

## Runtime data path

```text
MPU6050 internal sampler @ 1 kHz
        |
        +-- accel XYZ + gyro XYZ -> hardware FIFO (12 bytes/sample)
        +-- DATA_RDY -> MPU_INT
                         |
                         | optional TRC-V1.0 jumper
                         v
                       IO21
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

## TRC-V1.0 hardware note

The schematic names MPU6050 pin 12 `MPU_INT`, but that net is not routed to an ESP32 GPIO. GPIO21 is exposed on P4 and otherwise unused by the current runtime, so firmware configures IO21 as the bring-up DRDY input with an internal pull-down.

To validate hardware DRDY, add a temporary jumper:

```text
MPU6050 pin 12 / MPU_INT  ->  ESP32 IO21 / P4 IO21
```

Without that jumper the FIFO + asynchronous-I2C path still operates from the existing sensor-generation requests. DRDY statistics remain zero/fallback and make the missing physical connection explicit. Do not interpret an unmodified board as DRDY-driven.

## Why not DMP

The Balance control path remains based on the repository-owned 1 kHz estimator. The DMP can be evaluated later as an observer, but the current control path avoids replacing the 1 kHz raw inertial stream with the lower-rate DMP output.

## Validation targets

After flashing, repeat `twtool diag realtime-check`. Compare the previous direct-register path against FIFO/async I2C using:

- sensor read/join failures;
- completion cadence;
- MPU transfer profile;
- Core-1 execution time and period jitter;
- sensor freshness during short Balance trials.

The next sensor-domain cut is to make a validated MPU_INT edge the generation clock rather than merely observing it alongside the existing request clock.
