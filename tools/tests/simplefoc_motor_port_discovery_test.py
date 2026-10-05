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
assert args.scan_attempts == 3
args = module._parser().parse_args(["COM32"])
assert args.port == "COM32"


class FakeSerialException(Exception):
    pass


class FakePort:
    def __init__(self, device: str) -> None:
        self.port = device
        self.writes: list[bytes] = []
        self.closed = False

    def reset_input_buffer(self) -> None:
        raise AssertionError("discovery must not reset the RFCOMM input buffer")

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def flush(self) -> None:
        raise AssertionError("discovery must not flush the RFCOMM output path")

    def readline(self) -> bytes:
        if self.writes:
            return b"sfoc_motor_status,busy=0,connected=1\r\n"
        return b""

    def close(self) -> None:
        self.closed = True


class FakeSerialModule:
    SerialException = FakeSerialException

    def __init__(self) -> None:
        self.open_calls = 0
        self.last_port: FakePort | None = None

    def Serial(
        self,
        device: str,
        baud: int,
        timeout: float,
        write_timeout: float,
    ) -> FakePort:
        assert baud == 115200
        assert timeout == 0.25
        assert write_timeout == 0.5
        self.open_calls += 1
        if self.open_calls == 1:
            raise FakeSerialException("OSError(22, signal wait timeout, None, 121)")
        self.last_port = FakePort(device)
        return self.last_port


fake = FakeSerialModule()
original_sleep = module.time.sleep
module.time.sleep = lambda _: None
try:
    selected, status = module._probe_status(fake, "COM32", 115200, 0.05, 3)
finally:
    module.time.sleep = original_sleep

assert fake.open_calls == 2
assert selected is fake.last_port
assert status is not None and status.startswith("sfoc_motor_status,")
assert fake.last_port is not None
assert fake.last_port.writes == [b"status\r\n"]

print("SimpleFOC motor port discovery helpers: PASS")
