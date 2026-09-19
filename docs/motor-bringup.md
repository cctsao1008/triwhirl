# Motor bring-up plan

## Objective

Establish the actuator coordinate system and the usable `Vq` path before attitude estimation or balance control work begins.

## Stage M0: software-only verification

The host-testable three-phase field generator must satisfy:

1. all phase commands stay inside `[0, voltage_limit]`;
2. the three phase commands remain balanced around the same center voltage;
3. excessive requested amplitude is clamped;
4. invalid floating-point input collapses to the board's zero line-to-line command rather than producing an uncontrolled phase command.

Wheel kinematics are also exercised on the host. The tests cover forward/reverse AS5600 wrap, `micros()` timer wrap, zero-delta-time rejection, direction-ambiguous half-turn samples, and first-order velocity filtering.

## Bridge stopped-field behavior

The PCB ties each `Moto_INx` net to both EG2133 `HINx` and active-low `LINx#`. A driven low selects the low-side MOSFET and a driven high selects the high-side MOSFET. There is no separate phase-enable signal on this board.

`stop` and the boot state command all three PWM duties to zero. This is the **low-side zero vector**: all three motor phases are tied to the low rail. The commanded line-to-line motor voltage is zero, but the bridge is not high impedance. A hand-spun reaction wheel may therefore show dynamic-braking drag. This behavior is expected from the board topology and should not be mistaken for an encoder or bearing fault.

The same distinction applies to software fallback paths that emit `{0,0,0}`: they remove commanded line-to-line drive, but they do not electrically disconnect the motor.

## Stage M1: AS5600 sanity

The firmware reads the AS5600 directly over the board's dedicated I2C bus. Mechanical position uses the sensor's 12-bit `RAW ANGLE` output (`0x0C`/`0x0D`); magnet health uses `STATUS` (`0x0B`).

The initial bring-up sampler requests angle data at 1 kHz. Wheel speed is derived from timestamped modular count differences and an initial 10 ms first-order low-pass filter. These are bring-up settings, not final control-loop parameters; real hardware logs determine whether they should change.

With the rotating-field command stopped:

1. run `status` and confirm the I2C read and magnetic-status fields are valid;
2. rotate the reaction wheel slowly through repeated forward and reverse turns, allowing for the zero-vector braking drag described above;
3. verify `raw` wraps at the 12-bit boundary while `unwrapped_count` remains continuous;
4. verify the signs of `vel_inst_rad_s` and `vel_rad_s` for both directions;
5. inspect noise while stationary and at several hand-driven speeds;
6. record any I2C read errors or invalid velocity samples.

The bring-up telemetry header is printed at boot. Relevant encoder fields are:

```text
status_ok
sample_ok
mag
raw
unwrapped_count
angle_rad
unwrapped_rad
vel_rad_s
vel_inst_rad_s
vel_valid
read_errors
```

The integer `unwrapped_count` is the preferred long-duration multi-turn diagnostic. The floating-point unwrapped angle is convenient for short experiments but is not the authoritative long-run turn counter.

## Stage M2: low-voltage rotating electrical field

The bring-up firmware provides:

```text
field <electrical_hz> <amplitude_v>
stop
status
```

`field` is accepted only after a successful AS5600 angle read and healthy magnet status. Start at low electrical frequency and low amplitude. Boot state is the stopped-field zero vector described above.

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
2. link the AS5600 through the project sensor path;
3. run controlled electrical-zero/alignment;
4. use voltage torque mode;
5. characterize positive/negative `Vq`, wheel acceleration, steady speed, dead zone, and bus sag.

The final higher-level control input remains commanded `Vq`; there is no phase-current loop on this hardware.
