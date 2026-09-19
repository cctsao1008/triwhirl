from __future__ import annotations

from datetime import datetime


def host_timestamp() -> str:
    """Return a local, offset-aware wall-clock timestamp for host observability."""
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def host_print(message: object = "") -> None:
    """Print one host-side line with an explicit receipt/observation timestamp."""
    print(f"[host {host_timestamp()}] {message}")
