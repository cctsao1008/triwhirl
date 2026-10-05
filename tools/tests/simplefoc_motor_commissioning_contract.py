from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "main" / "runtime_simplefoc_motor_commissioning.cpp"
text = SOURCE.read_text(encoding="utf-8")

required = [
    '#include "esp32-hal-alloc-bt-classic-mem.h"',
    '#include "esp32-hal-bt.h"',
    'btStartMode(BT_MODE_CLASSIC_BT)',
    'actuator=DISABLED,autostart=0',
]

missing = [token for token in required if token not in text]
if missing:
    raise SystemExit(
        "SimpleFOC motor commissioning contract violation; missing: "
        + ", ".join(missing)
    )

print("SimpleFOC motor commissioning contract: PASS")
