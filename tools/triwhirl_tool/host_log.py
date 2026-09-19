from __future__ import annotations

from datetime import datetime


def _local_now() -> datetime:
    return datetime.now().astimezone()


def host_stamp() -> str:
    now = _local_now()
    return f"[{now.strftime('%H:%M:%S.%f')[:-3]}]"


def host_print(*items: object, sep: str = " ", end: str = "\n") -> None:
    message = sep.join(str(item) for item in items)
    print(f"{host_stamp()} {message}", end=end)


def print_session_header() -> None:
    now = _local_now()
    offset = now.strftime("%z")
    if len(offset) == 5:
        offset = f"{offset[:3]}:{offset[3:]}"
    print(f"session {now.strftime('%Y-%m-%d')} {offset}")
