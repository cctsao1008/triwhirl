# Vendor reference control architecture

This note records control-architecture evidence from the seller-provided `TRC-V1.1` source archive. It is reference material, not a direct code/gain port.

## Three upright vertices are one periodic control coordinate

The vendor sketch computes its pendulum error with a 120-degree periodic reduction:

```cpp
float pendulum_angle = constrainAngle(fmod(compAngleZ,120)-target_angle);
```

and `constrainAngle()` folds the result into the nearest local upright sector. The practical interpretation is:

```text
theta_error = wrap_periodic(theta - theta_ref, 120 deg)
```

so the three physical upright vertices map into the same local balance coordinate. The controller is therefore not A/B/C-specific.

TriWhirl uses the same contract through `triwhirl::periodicUprightErrorDeg()` / `periodicUprightErrorRad()`.

Named A/B/C vertices remain useful for logging, identification provenance, and measuring real inter-vertex asymmetry, but they are not separate controller identities.

## Vendor balance state and actuator architecture

The vendor balance call is structurally:

```cpp
target_velocity = controllerLQR(
    pendulum_angle,
    -Gyro,
    motor.shaftVelocity());
```

Thus its balance state is the same three-state shape used by TriWhirl:

```text
[local body angle error, body angular rate, reaction-wheel speed]
```

However, the vendor output is a **wheel velocity target** followed by SimpleFOC velocity PI, while TriWhirl's robust-control architecture is designed to command `Vq` directly:

```text
vendor:   state feedback -> wheel velocity target -> velocity PI -> Vq
TriWhirl: H-infinity state feedback ------------------------------> Vq
```

Therefore the vendor LQR gains are not dimensionally compatible with TriWhirl's direct-`Vq` gain and must not be copied.

## Balance / swing-up regions

The vendor code switches behavior by the magnitude of the periodic local angle error: a small region uses balance control and larger regions use swing-up voltage commands. This supports TriWhirl's separation between local upright control and a supervisory capture/recovery mechanism.

The exact vendor thresholds and voltages are tuning data for that implementation, not TriWhirl requirements.

## Reaction-wheel momentum unloading

The vendor LQR logic also slowly adjusts `target_angle` after the system has remained stable when reaction-wheel speed stays biased. Functionally this is a slow equilibrium-bias loop:

```text
wheel momentum bias
    -> small bounded upright-reference bias
    -> gravity supplies a counter-torque
    -> wheel speed is unloaded
```

This is a useful architecture reference for TriWhirl issue #18. It should be implemented as a slow, bounded outer bias around the same 120-degree periodic equilibrium, while the fast H-infinity loop remains unchanged.

The seller's numeric wheel-speed threshold, angle increment, and timing are not adopted without measurement.

## Timing evidence

The vendor Arduino sketch performs MPU6050 I2C work, SimpleFOC operations, control logic, and serial/Wi-Fi activity without an explicit deterministic 1 kHz scheduler. Its reported real-world ability to balance is evidence that first upright capture does not inherently require an exact 1 kHz loop.

This does **not** make TriWhirl's measured runtime jitter irrelevant. TriWhirl should still characterize and harden its actual control cadence before claiming robust-control guarantees. The vendor implementation only argues against making an exact 1 kHz scheduler a prerequisite for the first closed-loop standing test.
