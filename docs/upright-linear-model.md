# Upright linear model

## Control-coordinate contract

TriWhirl has three physically valid upright contact vertices separated by 120 degrees in the IMU frame. The balance controller is **not** tied to one named vertex.

For control, the natural coordinate is 120-degree periodic rather than a vertex-specific state:

```text
theta_error = wrap_periodic(theta - theta_ref, period = 120 deg)
            in [-60 deg, +60 deg)

x = [theta_error, theta_rate, wheel_rate]^T
        -> one shared balance controller
        -> Vq
```

With the current IMU-frame reference near 68 degrees, the measured upright centers are approximately:

```text
A =  +68 deg
B =  -52 deg = 68 - 120
C = -172 deg = 68 - 240
```

All three therefore map to `theta_error = 0` under the same periodic coordinate. A/B/C classification remains useful for logging and identification provenance, but the runtime balance law does not need a different gain or a different state definition for each named vertex.

The firmware contract is implemented by `triwhirl::periodicUprightErrorDeg()` / `periodicUprightErrorRad()` in `components/triwhirl_core/include/triwhirl/upright_geometry.hpp`. Matching host helpers live in `tools/triwhirl_tool/geometry.py`.

## Seller reference implementation

The seller-provided TRC-V1.1 source independently confirms this control structure on the same class of hardware. Its balance path first reduces the fused body angle modulo 120 degrees and then wraps the result into roughly `[-60,+60]` before applying one set of balance gains. It does not run three independent balance controllers for three corners.

That implementation also provides two useful architectural references:

- coarse swing-up is separate from the near-upright balance law;
- after balancing, it slowly trims the target angle according to persistent reaction-wheel speed, effectively using equilibrium-bias adjustment for momentum management.

The seller controller is **not** copied into TriWhirl: it drives a SimpleFOC velocity loop with LQR-like outer gains, whereas TriWhirl's planned robust controller commands `Vq` directly. Its value here is evidence for the 120-degree periodic coordinate and for the usefulness of a slow momentum-unloading reference trim, not as a source of transferable gain values.

A detailed source-derived comparison is kept in `docs/vendor-reference-control.md`.

## Identification versus control coverage

Plant identification does not need complete, symmetric data from all three physical vertices before a shared controller can be synthesized.

A data campaign may use:

- one vertex: nominal local plant only;
- two vertices: nominal plant plus measured inter-vertex variation;
- three vertices: broader direct sampling of the local plant family.

Per-vertex fits should still be computed separately first. This preserves visibility into excitation quality, conditioning, residuals, and real asymmetry. A selected subset may then be promoted into a shared nominal/uncertainty model explicitly; vertices must not be pooled silently.

## Identified local state-space form

For a selected upright vertex, with equilibrium-centered local coordinates:

```text
x = [theta_error, theta_rate, wheel_rate]^T
u = Vq

x_dot = A x + Bv u + d
```

with

```text
A = [ 0        1        0      ]
    [ a_theta  a_rate   a_wheel]
    [ c_theta  c_rate   c_wheel]

Bv = [0, b_vq, d_vq]^T
```

The fitted affine term `d` is retained as diagnostic evidence. Robust synthesis should center the equilibrium and treat residual bias as disturbance/model error rather than embedding a permanent feed-forward term without validation.

## Pipeline

1. Extract probe windows from authoritative TWLG.
2. Fit A/B/C independently with `tools/parameter_id/body_active_fit.py`.
3. Select one, two, or three usable vertex fits.
4. Convert them to state-space with `model/linearization/from_active_fit.py`.
5. Build the compact measured polytope with `model/uncertainty/build_polytopic.py`.
6. Synthesize one common robust state-feedback gain over that plant set.
7. At runtime apply that one gain to the 120-degree-periodic local state.

Example using two measured locations:

```powershell
python tools/parameter_id/body_active_fit.py `
  artifacts/swing-native-04-active.csv `
  --derivative-window 3 `
  -o artifacts/swing-native-04-fit.json

python model/linearization/from_active_fit.py `
  artifacts/swing-native-04-fit.json `
  --vertices B,C `
  --nominal B `
  --provenance-note "swing-native-04 contained human swing assist; local probe fits are provisional" `
  -o artifacts/swing-native-04-linear.json

python model/uncertainty/build_polytopic.py `
  artifacts/swing-native-04-linear.json `
  -o artifacts/swing-native-04-polytope.json
```

## Current evidence policy

A run with human assistance during coarse swing-up is not evidence of autonomous swing-up. Its short local probe windows may still be usable for plant identification if no external contact occurs during those windows, but such fits remain provisional until the provenance is reviewed. The distinction must remain explicit in generated artifacts and issue notes.

The controller's supported upright vertices are a control/runtime property. The number of vertices used for identification is a data-quality decision. These are deliberately separate concepts.
