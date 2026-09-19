# Local parameter identification

TriWhirl separates realtime experiment execution from host-side acquisition, decoding, fitting, and analysis.

The canonical host entry point is now the TriWhirl Toolbox:

```powershell
python tools/twtool.py --help
```

The older scripts in this directory remain as compatibility backends/wrappers while their reusable logic is moved into `tools/triwhirl_tool/`.

## Realtime boundary

BLE is not a realtime control or measurement authority. The ESP32 control loop runs at 1 kHz and the firmware TWLG logger snapshots the state in that same timing domain. Timing-critical excitation, vertex detection, and capture decisions must ultimately execute on the ESP32 rather than on the Windows/Python/BLE round trip.

The current binary logging path is:

```text
ESP32 1 kHz control/state
        |
        +-- 32-byte synchronized TWLG record
                |
             SRAM buffer
                |
        non-critical flash writer
                |
          dedicated TWLG partition
                |
        experiment completes
                |
          BLE bulk download
                |
              host
```

Use the toolbox for post-run transfer and inspection:

```powershell
python tools/twtool.py log download -o artifacts/run-01.twlog
python tools/twtool.py log inspect artifacts/run-01.twlog
python tools/twtool.py log decode artifacts/run-01.twlog -o artifacts/run-01.csv
```

`log download` validates the completed binary transfer before saving it. `log inspect` validates the TWLG header/CRC and reports sample rate, duration, body-angle range, body/wheel rates, Vq range, dropped records, and fault coverage. `log decode` converts the validated 32-byte records to CSV.

## Three upright vertices

The Reuleaux body has three legitimate upright vertex equilibria separated by 120 body degrees. Identification therefore treats contact mode as part of the plant state rather than assuming one valid upright orientation.

Current IMU-frame naming anchors are approximately:

```text
A ~=  +68 deg
B ~=  -52 deg
C ~= -172 deg   (equivalent to +188 deg)
```

These are classification centers, not a claim of exact dynamic symmetry. PCB, battery, motor, and wheel mass distribution can make the three local plants differ. Active fits are kept separate as A/B/C and can later form a nominal-plus-uncertainty or polytopic robust-control model.

The shared geometry implementation now lives in `tools/triwhirl_tool/geometry.py`; `vertex_geometry.py` is a compatibility wrapper for older scripts.

## Toolbox command map

Identification acquisition:

```powershell
python tools/twtool.py id actuator-uart ...
python tools/twtool.py id actuator-ble ...
python tools/twtool.py id body-free ...
python tools/twtool.py id body-local ...
python tools/twtool.py id body-active ...
python tools/twtool.py id swing ...
```

Model fitting:

```powershell
python tools/twtool.py fit actuator artifacts/local-id.csv -o artifacts/local-fit.json
python tools/twtool.py fit body-local artifacts/body-local.csv -o artifacts/body-local-fit.json
python tools/twtool.py fit body-active artifacts/body-active.csv -o artifacts/body-active-fit.json
```

The identification/fitting commands currently route to the established scripts while migration continues. This preserves existing command-line options and dataset compatibility.

## Legacy BLE acquisitions

The existing BLE acquisition scripts remain useful for historical datasets and low-rate/manual experiments, but their host timing must not be interpreted as deterministic realtime timing.

- `acquire_ble.py`: generic untethered Vq profile acquisition.
- `body_free_ble.py`: zero-actuation free-body response.
- `body_local_ble.py`: passive local-upright release acquisition.
- `body_active_ble.py`: pre-armed local active release acquisition.
- `auto_swing_id_ble.py`: host-driven autonomous rocking experiment used during development of the swing concept.

The host-driven swing tool demonstrated that the reaction wheel can pump the body through the A/B/C upright regions, but BLE command latency is too large relative to the roughly tens-of-milliseconds local unstable dynamics for it to be the final identification timing path. Do not treat host command arrival time as plant input time.

## Telemetry / fit conventions

Legacy CSV telemetry schema v2 contains the firmware state fields used by the fitters. The measured firmware `vq_v` is the authoritative input, not host send time.

The preliminary actuator/local model uses:

```text
theta_ddot = a1*theta + a2*theta_rate + a3*wheel_rate + b1*Vq + c1
wheel_accel = a4*theta + a5*theta_rate + a6*wheel_rate + b2*Vq + c2
```

Active upright fits classify trials by A/B/C vertex and never pool different vertices automatically. A fit is only a synthesis candidate when its sample coverage, signed excitation, rank, local-angle coverage, and Vq coefficient significance requirements are met; otherwise it remains diagnostic evidence.

## Dependencies

Install host dependencies once in the active Python environment:

```powershell
python -m pip install -r tools/parameter_id/requirements.txt
```

This currently provides NumPy, pySerial, and Bleak for the legacy acquisition/fitting backends and toolbox BLE transfer.
