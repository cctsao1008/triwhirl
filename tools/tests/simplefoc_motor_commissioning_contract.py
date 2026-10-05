from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
APP = ROOT / "commissioning" / "simplefoc_motor_app_main.cpp"
RUNTIME = ROOT / "main" / "runtime_simplefoc_motor_commissioning.cpp"

app = APP.read_text(encoding="utf-8")
runtime = RUNTIME.read_text(encoding="utf-8")

required_app = [
    '#include "esp32-hal-alloc-bt-classic-mem.h"',
    'runSimpleFocMotorCommissioning()',
]
required_runtime = [
    'btStartMode(BT_MODE_CLASSIC_BT)',
    'actuator=DISABLED,autostart=0',
    'kMaxCommissioningVoltageV = 0.5F',
    'kMaxTargetVelocityRadS = 5.0F',
]

missing = [token for token in required_app if token not in app]
missing += [token for token in required_runtime if token not in runtime]
if missing:
    raise SystemExit(
        "SimpleFOC motor commissioning contract violation; missing: "
        + ", ".join(missing)
    )

print("SimpleFOC motor commissioning contract: PASS")
