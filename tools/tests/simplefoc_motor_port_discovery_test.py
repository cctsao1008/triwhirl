from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))

from triwhirl_tool.commands import sfoc_motor as module  # noqa: E402

args = module._parser().parse_args([])
assert args.name == "TriWhirl"
assert args.address is None
assert args.scan_timeout == 10.0
assert not hasattr(args, "port")
assert not hasattr(args, "baud")
assert not hasattr(args, "scan_attempts")

values = module._parse_result(
    "sfoc_motor_result,init_ok=1,aborted=0,backend_faulted=0,"
    "sensor_valid=1,pos_mean_rad_s=4.2,neg_mean_rad_s=-4.0"
)
assert values["init_ok"] == "1"
assert module._float(values, "pos_mean_rad_s") == 4.2
assert module._float(values, "neg_mean_rad_s") == -4.0

source = (TOOLS / "triwhirl_tool" / "commands" / "sfoc_motor.py").read_text(
    encoding="utf-8"
)
required = [
    "discover_target(",
    "BleakClient(target)",
    "client.start_notify(TX_UUID, transport.on_notify)",
    'await transport.send("status")',
    'prefixes=("sfoc_motor_status,",)',
    "BLE GATT command path proven before motor init",
]
forbidden = [
    "pyserial",
    "serial.tools",
    "BTHENUM",
    "COM32",
    "_bluetooth_candidates",
]
missing = [token for token in required if token not in source]
present_forbidden = [token for token in forbidden if token in source]
if missing or present_forbidden:
    raise SystemExit(
        "SimpleFOC motor BLE host contract violation; "
        + ("missing: " + ", ".join(missing) if missing else "")
        + ("; " if missing and present_forbidden else "")
        + ("forbidden: " + ", ".join(present_forbidden) if present_forbidden else "")
    )

print("SimpleFOC motor BLE host helpers: PASS")
