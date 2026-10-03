# Pure-fuzzy near-upright SITL

This note records the first closed-loop software-in-the-loop path for the target pure-fuzzy attitude architecture. It is a **simulation contract**, not a hardware tune.

## Command semantics

The full-fuzzy controller emits an **absolute wheel `target_velocity [rad/s]`**. It does not emit torque, `Vq`, wheel acceleration, duty cycle, or PWM.

That distinction matters for the wheel-velocity input. At zero attitude error and zero body rate, an already-spinning wheel should not automatically be commanded in the opposite direction merely because momentum is nonzero. The first qualitative rule seed therefore uses the wheel linguistic term as the target-velocity baseline and shifts that baseline according to attitude restoring/damping urgency.

At the rule centers:

```text
ZE theta_error + ZE theta_rate + wheel term
        -> same wheel term as target baseline
```

Positive attitude/rate urgency shifts the target in the negative correcting direction; negative urgency shifts it in the positive direction. The 125-rule surface remains bounded and exactly odd under mirrored state.

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

These values were selected only to exercise the complete software path on the existing historical nominal local fixture. They are **not**:

- firmware commissioning values;
- validated hardware normalization scales;
- SimpleFOC PI/LPF settings;
- motor capability limits;
- global swing-up authority;
- proof that the real TriWhirl will balance with these values.

Hardware normalization, motor/output polarity, real target-velocity limits, and SimpleFOC velocity-loop behavior still require independent bench evidence.

No LQR, PID, or H-infinity controller is part of this path.
