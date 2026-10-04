# Pure-fuzzy near-upright SITL

This note records the first software-in-the-loop command path for the target pure-fuzzy attitude architecture. It is a **structural simulation contract**, not a hardware tune and not currently a closed-loop recovery authority.

## Command semantics

The full-fuzzy controller emits an **absolute wheel `target_velocity [rad/s]`**. It does not emit torque, `Vq`, wheel acceleration, duty cycle, or PWM.

That distinction matters for the wheel-velocity input. At zero attitude error and zero body rate, an already-spinning wheel should not automatically be commanded in the opposite direction merely because momentum is nonzero. The first qualitative rule seed therefore uses wheel velocity as the absolute target-velocity baseline and shifts that baseline according to attitude restoring/damping urgency.

The wheel input and target output have independent normalization scales. Therefore an identity in linguistic coordinates is not automatically an identity in physical rad/s. The seed explicitly converts the wheel linguistic center into normalized target coordinates with

```text
wheel_velocity_scale_rad_s / target_velocity_limit_rad_s
```

so, away from target saturation:

```text
theta_error = 0
theta_rate  = 0
        -> target_velocity [rad/s] = wheel_velocity [rad/s]
```

This scale conversion is part of the command coordinate, not a plant/controller gain. Positive attitude/rate urgency shifts the target in the negative correcting direction; negative urgency shifts it in the positive direction. The 125-rule surface remains bounded and exactly odd under mirrored state.

Momentum unloading is a later full-fuzzy behavior. It must account for attitude headroom / wheel-speed saturation context instead of being approximated by an unconditional command opposite to wheel velocity.

## SITL path

`tools/sitl/pure_fuzzy_balance_sitl.cpp` exercises:

```text
[theta_error, theta_rate, wheel_velocity]
                  |
                  v
       FuzzyBalanceController
                  |
                  v
        target_velocity [rad/s]
                  |
                  v
         VelocityServoModel
                  |
                  v
      wheel_accel_command [rad/s^2]
                  |
                  v
   historical nominal local fixture
```

The executable now uses a short horizon to check unit consistency, finite/bounded composition, attitude-command direction, and mirrored response. It intentionally does **not** claim long-horizon balance or disturbance recovery.

## Why recovery authority was withdrawn

The first #67 SITL seed matched the wheel linguistic input directly to the normalized target output. With different physical scales (`17 rad/s` wheel input scale versus `80 rad/s` target limit), that made

```text
normalized target = normalized wheel velocity
```

but did **not** make

```text
target_velocity [rad/s] = wheel_velocity [rad/s].
```

The resulting extra wheel-speed feedback happened to make the historical nominal fixture recover in the earlier simulation. That recovery result depended on a unit/coordinate mismatch and is therefore withdrawn as control evidence.

After correcting the wheel baseline with the `17/80` scale ratio, the same untuned qualitative seed no longer stabilizes the historical nominal fixture over the previous five-second horizon. We do not retune the fuzzy law merely to rescue that provisional historical fit. Until supported plant/tuning evidence exists, CI treats this executable as a structural path test only and reports:

```text
pure_fuzzy_sitl_authority=STRUCTURAL_ONLY_HISTORICAL_NOMINAL
closed_loop_recovery_authority=NONE
hardware_tune_authority=NONE
```

## Authority boundary

The current simulation fixture uses:

```text
theta_error_scale      = 0.12 rad
theta_rate_scale       = 1.30 rad/s
wheel_velocity_scale   = 17.0 rad/s
target_velocity_limit  = 80.0 rad/s
servo tau              = 0.10 s
servo accel limit      = 150 rad/s^2
```

The qualitative seed is generated with the corresponding `17 / 80` wheel-input-to-target-output normalization ratio so the zero-attitude wheel baseline remains physically consistent in rad/s.

These values were selected only to exercise the complete software path on the existing historical nominal local fixture. They are **not**:

- firmware commissioning values;
- validated hardware normalization scales;
- SimpleFOC PI/LPF settings;
- motor capability limits;
- local closed-loop recovery authority;
- global swing-up authority;
- proof that the real TriWhirl will balance with these values.

Hardware normalization, motor/output polarity, real target-velocity limits, momentum-management behavior, and SimpleFOC velocity-loop behavior still require independent bench evidence.

No LQR, PID, or H-infinity controller is part of this path.
