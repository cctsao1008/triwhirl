# Motor bring-up plan

## Objective

Establish the actuator coordinate system and the usable `Vq` path before attitude estimation or balance control work begins.

## Stage M0: software-only verification

The host-testable three-phase field generator must satisfy:

1. all phase commands stay inside `[0, voltage_limit]`;
2. the three phase commands remain balanced around the same center voltage;
3. excessive requested amplitude is clamped;
4. invalid floating-point input fails passive.

## Stage M1: AS5600 sanity

With motor excitation disabled:

1. rotate the reaction wheel by hand;
2. verify continuous angle updates;
3. verify sign convention;
4. observe velocity noise and wrap behavior.

## Stage M2: low-voltage rotating electrical field

The bring-up firmware provides:

```text
field <electrical_hz> <amplitude_v>
stop
status
```

Start at low electrical frequency and low amplitude. Boot state is always stopped.

This stage deliberately does not require a pole-pair assumption.

## Stage M3: identify pole pairs

Once the rotor tracks the rotating field approximately synchronously, estimate:

```text
pole_pairs ~= electrical_frequency / mechanical_revolutions_per_second
```

Repeat at several low frequencies and both directions. Accept the value only if the ratio clusters tightly around an integer.

## Stage M4: sensor-based SimpleFOC

Only after M1-M3 establish encoder direction, phase behavior, and pole pairs:

1. construct `BLDCMotor` with the measured pole-pair count;
2. link the AS5600 sensor;
3. run controlled electrical-zero/alignment;
4. use voltage torque mode;
5. characterize positive/negative `Vq`, wheel acceleration, steady speed, dead zone, and bus sag.

The final higher-level control input remains commanded `Vq`; there is no phase-current loop on this hardware.
