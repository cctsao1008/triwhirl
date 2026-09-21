# Observed TRC-V1.0 production board

This note records physical observations from the user's actual TriWhirl board. It intentionally overrides unverified connector assumptions taken from the vendor schematic.

## What the photographs establish

- The PCB silkscreen is `TRC-V1.0`.
- The physical board does **not** have the schematic's 2x10 `P4` header populated or visible as a matching footprint.
- The back side has small bed-of-nails / test-pad arrays, but their nets are not identified from photographs alone.
- A 24-pin QFN device consistent with the MPU6050 package is visible on the component side near the large 1000-uF capacitor. The package orientation appears to have a pin-1 marker, but individual hidden/inner copper routing cannot be proven from the photographs.

## Vendor-source evidence

The vendor V1.1 firmware uses:

```text
MPU6050 SDA = GPIO19
MPU6050 SCL = GPIO18
AS5600  SDA = GPIO23
AS5600  SCL = GPIO5
motor PWM   = GPIO33 / GPIO25 / GPIO32
```

It polls the MPU6050 14-byte sensor register window and does not configure or consume MPU_INT. GPIO21 is unused by the vendor source. This does not prove the PCB interrupt route, but it makes GPIO21 a reasonable non-destructive probe candidate.

## MPU6050 DRDY policy

The vendor schematic labels MPU6050 pin 12 as `MPU_INT`, but the actual PCB routing remains unverified. Firmware therefore does **not** make GPIO21 a scheduling authority. Instead it may configure GPIO21 as a passive rising-edge probe with no pull-up/down:

```text
MPU6050 INT routing        = UNKNOWN
kMpu6050IntGpio            = -1
kMpu6050IntProbeGpio       = 21
kMpu6050IntRoutingVerified = false
```

Observed edges are diagnostics only. A stable edge rate near the configured MPU6050 1 kHz DATA_RDY rate would strongly support the hypothesis; zero or unrelated-rate activity would reject it. FIFO + asynchronous I2C remain valid independent of this probe.

The preferred software-only check is:

```text
python tools/twtool.py diag drdy-probe 5
```

The command snapshots `imu status`, observes for the requested interval, then reports the **counter delta** rather than a lifetime count. A result such as:

```text
drdy_probe,state=CANDIDATE_1KHZ,gpio=21,probe_only=1,seconds=5.000,edges=5001,rate_hz=1000.200,...
DRDY_PROBE_MATCH
```

is strong evidence that GPIO21 receives MPU6050 DATA_RDY, but it still does not grant the pin realtime scheduling authority. Use `--require-1khz` when the probe is part of an automated hardware acceptance check.

## How to establish the interrupt route

Preferred order:

1. first use the passive GPIO21 edge counter and compare its rate with the configured 1 kHz DATA_RDY rate;
2. if the signature is convincing, confirm with continuity or an oscilloscope when practical;
3. only then assign `kMpu6050IntGpio` and promote DRDY to sensor-generation authority.

Do not solder a jumper to an assumed GPIO solely from the vendor schematic.
