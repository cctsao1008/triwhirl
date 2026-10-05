from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = ROOT / "tools" / "triwhirl_tool" / "commands" / "sfoc_motor.py"

spec = spec_from_file_location("sfoc_motor", MODULE_PATH)
assert spec is not None and spec.loader is not None
module = module_from_spec(spec)
spec.loader.exec_module(module)


class PortInfo:
    def __init__(self, device: str, hwid: str) -> None:
        self.device = device
        self.hwid = hwid


ports = [
    PortInfo("COM5", r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}_VID&000102B0_PID&0000"),
    PortInfo("COM28", "USB VID:PID=1A86:7523"),
    PortInfo("COM32", r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}_LOCALMFG&0000"),
    PortInfo("COM31", r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}_LOCALMFG&0002"),
]

assert module._bluetooth_candidates(ports) == ["COM32", "COM31", "COM5"]
assert module._bluetooth_candidates([PortInfo("COM28", "USB VID:PID=1A86:7523")]) == []

args = module._parser().parse_args([])
assert args.port is None
args = module._parser().parse_args(["COM32"])
assert args.port == "COM32"

print("SimpleFOC motor port discovery helpers: PASS")
