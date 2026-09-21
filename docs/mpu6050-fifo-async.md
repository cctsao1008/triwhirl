# MPU6050 FIFO + asynchronous I2C bring-up

TriWhirl's MPU6050 runtime path uses the sensor FIFO and ESP-IDF asynchronous I2C transactions instead of reading the 14-byte live register window synchronously every control generation.

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
        +-- async FIFO_COUNT read
        +-- async FIFO_R_W burst
        |
        v
RuntimeSensorFrame -> Core 1 estimator / Balance
```

Temperature is intentionally excluded from FIFO because it is not part of the Balance state. All three gyro axes remain in FIFO so the runtime `imu map` command can still select the planar gyro axis without changing the hardware FIFO layout.

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

FIFO + asynchronous I2C remain active regardless of DRDY routing.

See `docs/hardware-observed-trc-v1.md` for physical-board evidence.

## Why not DMP

The Balance control path remains based on the repository-owned 1 kHz estimator. The DMP can be evaluated later as an observer, but the current control path avoids replacing the 1 kHz raw inertial stream with the lower-rate DMP output.

## Validation targets

After flashing, repeat `twtool diag realtime-check`. Compare the previous direct-register path against FIFO/async I2C using:

- sensor read/join failures;
- completion cadence;
- MPU transfer profile;
- Core-1 execution time and period jitter;
- sensor freshness during short Balance trials;
- passive DRDY edge rate on GPIO21.

DRDY becomes eligible as the generation clock only after the candidate route is promoted from passive probe to verified board mapping.
