# Observed TRC-V1.0 production board

This note records physical observations from the user's actual TriWhirl board. It intentionally overrides unverified connector assumptions taken from the vendor schematic.

## What the photographs establish

- The PCB silkscreen is `TRC-V1.0`.
- The physical board does **not** have the schematic's 2x10 `P4` header populated or visible as a matching footprint.
- The back side has small bed-of-nails / test-pad arrays, but their nets are not identified from photographs alone.
- A 24-pin QFN device consistent with the MPU6050 package is visible on the component side near the large 1000-uF capacitor. The package orientation appears to have a pin-1 marker, but individual hidden/inner copper routing cannot be proven from the photographs.

## Vendor-source evidence

The vendor `TRC-V1.1` firmware confirms the already-observed sensor buses (`MPU6050 SDA/SCL = GPIO19/18`, `AS5600 SDA/SCL = GPIO23/5`) but does not use `MPU_INT`, FIFO, DMP, or `attachInterrupt()`. GPIO21 is also unused by that firmware.

That does **not** prove that MPU6050 INT is physically routed to GPIO21 on the user's V1.0 PCB. It only makes GPIO21 a useful, non-conflicting hypothesis to probe.

## Consequence for MPU6050 DRDY

The vendor schematic labels MPU6050 pin 12 as `MPU_INT`, but the actual PCB routing of that net is currently unknown. Firmware therefore distinguishes an authoritative route from a passive hypothesis:

```text
MPU6050 INT authoritative routing = UNKNOWN
kMpu6050IntGpio                  = -1
kMpu6050IntRoutingVerified       = false
kMpu6050IntProbeGpio             = 21   # observation only
```

The GPIO21 probe is configured as an unbiased input and counts rising edges only. While `kMpu6050IntRoutingVerified == false`, those edges never wake the IMU worker, never become the sensor-generation clock, and never affect Balance admission or actuation.

FIFO + asynchronous I2C remain the active acquisition path regardless of the probe result.

## Firmware-assisted route test

`imu status` now reports:

```text
drdy_gpio
drdy_probe_only
drdy_edges
drdy_consumed
drdy_fallback_reads
```

`twtool diag realtime-check` resets the acquisition statistics at the beginning of the profiling window and classifies the passive GPIO21 evidence using the DRDY-edge / sensor-request ratio. A result near one edge per 1-kHz request is reported as `classification=MATCH`; zero edges is `NO_EDGES`; other rates are `INCONCLUSIVE`.

A `MATCH` is strong runtime evidence but is still not treated as an authoritative PCB net declaration by firmware. Continuity or scope verification remains the final hardware confirmation before promoting DRDY to scheduling authority.

## How to establish the interrupt route

Preferred order:

1. run `python tools/twtool.py diag realtime-check 5 --baseline-seconds 3` and inspect the `drdy_probe,...` line;
2. if GPIO21 reports a stable near-1-kHz `MATCH`, confirm with continuity or an oscilloscope when practical;
3. otherwise identify MPU6050 pin 12 from the package pin-1 orientation and probe the accessible test pads / ESP32 GPIO pads;
4. only after a physical net is verified, assign `kMpu6050IntGpio` and enable DRDY-driven scheduling.

Do not solder a jumper to an assumed GPIO from the vendor schematic until the actual PCB net is verified.
