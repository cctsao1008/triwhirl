from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from bleak import BleakClient

DEVICE_NAME = "TriWhirl"
SERVICE_UUID = "54f10000-8f4d-4f3a-b691-54524957484c"
RX_UUID = "54f10001-8f4d-4f3a-b691-54524957484c"
TX_UUID = "54f10002-8f4d-4f3a-b691-54524957484c"
DEFAULT_WRITE_CHUNK = 20


async def discover_target(
    *,
    name: str = DEVICE_NAME,
    address: str | None = None,
    scan_timeout: float = 10.0,
    verbose: bool = True,
) -> Any:
    """Resolve a TriWhirl BLE target without importing Bleak until needed."""
    if address:
        return address

    from bleak import BleakScanner

    if verbose:
        print(f"scanning for BLE device {name!r} ...")
    device = await BleakScanner.find_device_by_name(name, timeout=scan_timeout)
    if device is None:
        raise RuntimeError(f"BLE device {name!r} not found")
    if verbose:
        print(f"found {device.name or name}: {device.address}")
    return device


async def send_command(
    client: "BleakClient",
    command: str,
    *,
    chunk_size: int = DEFAULT_WRITE_CHUNK,
) -> None:
    """Send one newline-terminated firmware command over the native RX characteristic."""
    if chunk_size <= 0:
        raise ValueError("chunk_size must be positive")
    payload = (command.rstrip("\r\n") + "\n").encode("ascii")
    for offset in range(0, len(payload), chunk_size):
        await client.write_gatt_char(
            RX_UUID,
            payload[offset : offset + chunk_size],
            response=False,
        )


def drain_queue(queue: "asyncio.Queue[bytes]") -> None:
    while not queue.empty():
        queue.get_nowait()
