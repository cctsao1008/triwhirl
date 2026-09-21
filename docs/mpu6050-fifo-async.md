# MPU6050 FIFO acquisition bring-up

TriWhirl's MPU6050 runtime path uses the sensor FIFO instead of reading the 14-byte live register window every control generation.

The current validated baseline deliberately keeps the ESP-IDF I2C bus in synchronous mode during probe, configuration, and runtime FIFO reads. The Core-0 IMU acquisition worker owns those blocking transfers, so Core 1 remains free of blocking MPU I2C work.

An asynchronous callback implementation remains in the driver for later evaluation, but the platform helper no longer forces a non-zero `trans_queue_depth`. That forced async-bus configuration changed the behavior of the previously known-good synchronous probe/configuration path and caused MPU startup regression on the physical unit.

## Runtime data path

```text
MPU6050 internal sampler @ 1 kHz
        |
        +-- accel XYZ + gyro XYZ -> hardware FIFO (12 bytes/sample)
        +-- DATA_RDY -> MPU_INT
        |
        v
Core-0 IMU acquisition worker
        |
        +-- synchronous FIFO_COUNT read
        +-- synchronous FIFO_R_W burst
        |
        v
RuntimeSensorFrame -> Core 1 estimator / Balance
```

Temperature is intentionally excluded from FIFO because it is not part of the Balance state. All three gyro axes remain in FIFO so the runtime `imu map` command can still select the planar gyro axis without changing the hardware FIFO layout.

## Why synchronous I2C is the baseline

ESP-IDF uses a non-zero `trans_queue_depth` for asynchronous master transactions. TriWhirl originally enabled that queue globally in the shared bus helper before MPU probing and configuration. On the physical board this coincided with repeated `i2c_master_probe()` timeouts although the same board had worked with the earlier synchronous bus setup.

The recovery baseline therefore preserves the caller's bus configuration exactly. With `trans_queue_depth == 0`, MPU probe/configuration and FIFO transactions use the synchronous driver path. The MPU driver already treats async callback registration failure as non-fatal and remains in FIFO mode.

This is not a return to direct 14-byte register polling. FIFO remains mandatory; only the I2C completion mechanism is synchronous.

## GPIO21 passive probe

The vendor schematic and production-board photographs do not prove where MPU6050 pin 12 (`MPU_INT`) is routed. The vendor firmware does not consume MPU_INT, and GPIO21 is unused by that firmware. Therefore GPIO21 is treated only as a provisional passive probe candidate.

Firmware configures GPIO21 as a rising-edge input without pulls and counts edges only:

```text
kMpu6050IntGpio = -1
kMpu6050IntProbeGpio = 21
kMpu6050IntRoutingVerified = false
```

While routing is unverified, DRDY edges are diagnostic only. They do not wake the sensor worker, do not clock sensor-frame generation, and are not Balance authority.

After flashing, run:

```text
imu status
```

The response includes:

```text
drdy_gpio=<candidate>
drdy_probe_only=1
drdy_edges=<count>
drdy_consumed=0
drdy_fallback_reads=<count>
```

With DATA_RDY enabled at the current 1 kHz sensor rate, GPIO21 is a strong candidate only if the observed edge rate is close to 1000 Hz over a multi-second interval. A zero count or an unrelated rate rejects that hypothesis. Promotion to the acquisition clock still requires either electrical continuity or a stable expected DATA_RDY signature plus an explicit board-mapping decision.

FIFO acquisition remains active regardless of DRDY routing.

See `docs/hardware-observed-trc-v1.md` for physical-board evidence.

## Why not DMP

The Balance control path remains based on the repository-owned 1 kHz estimator. The DMP can be evaluated later as an observer, but the current control path avoids replacing the 1 kHz raw inertial stream with the lower-rate DMP output.

## Validation targets

After flashing, repeat `twtool diag realtime-check`. Compare the old direct-register path against the FIFO baseline using:

- sensor read/join failures;
- completion cadence;
- MPU transfer profile;
- Core-1 execution time and period jitter;
- sensor freshness during short Balance trials;
- passive DRDY edge rate on GPIO21.

DRDY becomes eligible as the generation clock only after the candidate route is promoted from passive probe to verified board mapping.
