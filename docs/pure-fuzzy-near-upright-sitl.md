# Pure-fuzzy near-upright SITL

This note records the first closed-loop software-in-the-loop path for the target pure-fuzzy attitude architecture. It is a **simulation contract**, not a hardware tune.

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

The executable checks small positive/negative body-angle disturbances, a body-rate disturbance, a wheel-rate disturbance, bounded target velocity, and mirrored closed-loop response.

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
- global swing-up authority;
- proof that the real TriWhirl will balance with these values.

Hardware normalization, motor/output polarity, real target-velocity limits, and SimpleFOC velocity-loop behavior still require independent bench evidence.

No LQR, PID, or H-infinity controller is part of this path.
