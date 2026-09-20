#!/usr/bin/env python3
"""Fail CI if runtime composition debt grows during issue #32/A2.

The active firmware still has one explicitly tolerated source-inclusion chain:

    runtime_control.cpp -> runtime_main.cpp -> app_main.cpp

and five symbol-renaming shims in runtime_main.cpp. A2 is removing those pieces
incrementally. Until they are gone, CI prevents new .cpp includes, new
symbol-renaming interception, a second application-entry owner, queue/task
mechanics from leaking back into realtime control, or UART/BLE byte polling from
returning to the realtime source.
"""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MAIN = ROOT / "main"

ALLOWED_CPP_INCLUDES = {
    ("runtime_control.cpp", "runtime_main.cpp"),
    ("runtime_main.cpp", "app_main.cpp"),
}

ALLOWED_RENAMING_DEFINES = {
    ("runtime_main.cpp", "app_main", "triwhirl_legacy_app_main"),
    ("runtime_main.cpp", "updateEncoder", "triwhirl_legacy_updateEncoder"),
    ("runtime_main.cpp", "updateImu", "triwhirl_legacy_updateImu"),
    ("runtime_main.cpp", "initEncoderBus", "triwhirl_legacy_initEncoderBus"),
    ("runtime_main.cpp", "initImuBus", "triwhirl_legacy_initImuBus"),
}

CONTROL_FORBIDDEN_QUEUE_MECHANICS = (
    '"freertos/queue.h"',
    "xQueueCreate(",
    "xQueueSend(",
    "xQueueReceive(",
    "xQueueOverwrite(",
)

CONTROL_FORBIDDEN_TRANSPORT_IO = (
    "uart_read_bytes(",
    "triwhirl::ble::read(",
)

CPP_INCLUDE_RE = re.compile(r'^\s*#\s*include\s+"([^"]+\.cpp)"', re.MULTILINE)
RENAME_DEFINE_RE = re.compile(
    r"^\s*#\s*define\s+(app_main|updateEncoder|updateImu|initEncoderBus|initImuBus)\s+(\w+)",
    re.MULTILINE,
)
APP_MAIN_RE = re.compile(r'extern\s+"C"\s+void\s+app_main\s*\(')


def fail(message: str) -> None:
    raise SystemExit(f"runtime architecture guard: {message}")


def main() -> None:
    cpp_files = sorted(MAIN.glob("*.cpp"))
    if not cpp_files:
        fail("no main/*.cpp sources found")

    cpp_includes: set[tuple[str, str]] = set()
    renaming_defines: set[tuple[str, str, str]] = set()
    app_main_sources: list[str] = []

    for path in cpp_files:
        text = path.read_text(encoding="utf-8")

        for included in CPP_INCLUDE_RE.findall(text):
            cpp_includes.add((path.name, included))

        for symbol, replacement in RENAME_DEFINE_RE.findall(text):
            renaming_defines.add((path.name, symbol, replacement))

        if APP_MAIN_RE.search(text):
            app_main_sources.append(path.name)

    unexpected_includes = cpp_includes - ALLOWED_CPP_INCLUDES
    if unexpected_includes:
        fail(f"unexpected .cpp inclusion(s): {sorted(unexpected_includes)}")

    unexpected_renames = renaming_defines - ALLOWED_RENAMING_DEFINES
    if unexpected_renames:
        fail(f"unexpected symbol-renaming shim(s): {sorted(unexpected_renames)}")

    # During the transition both app_main.cpp and runtime_main.cpp contain an
    # app_main definition, but the legacy one is renamed by the tolerated shim.
    # Do not allow a third source to acquire application-entry ownership.
    unexpected_entries = set(app_main_sources) - {"app_main.cpp", "runtime_main.cpp"}
    if unexpected_entries:
        fail(f"unexpected app_main owner(s): {sorted(unexpected_entries)}")

    control_text = (MAIN / "runtime_control.cpp").read_text(encoding="utf-8")
    leaked_queue_mechanics = [
        token for token in CONTROL_FORBIDDEN_QUEUE_MECHANICS if token in control_text
    ]
    if leaked_queue_mechanics:
        fail(
            "queue mechanics leaked into runtime_control.cpp: "
            f"{leaked_queue_mechanics}"
        )

    leaked_transport_io = [
        token for token in CONTROL_FORBIDDEN_TRANSPORT_IO if token in control_text
    ]
    if leaked_transport_io:
        fail(
            "UART/BLE input polling leaked into runtime_control.cpp: "
            f"{leaked_transport_io}"
        )

    supervisor_io = MAIN / "runtime_supervisor_io.cpp"
    if not supervisor_io.exists():
        fail("runtime_supervisor_io.cpp is missing")
    supervisor_text = supervisor_io.read_text(encoding="utf-8")
    for required in ("uart_read_bytes(", "triwhirl::ble::read(", "xQueueCreate("):
        if required not in supervisor_text:
            fail(f"supervisor I/O boundary is missing expected primitive: {required}")

    # Keep the known debt explicit. As A2 removes an item, delete it from the
    # allow-list in the same change; the test intentionally does not require all
    # allow-listed debt to remain present.
    print("runtime architecture guard: PASS")
    print(f"  cpp_includes={sorted(cpp_includes)}")
    print(f"  renaming_shims={sorted(renaming_defines)}")
    print(f"  app_main_sources={sorted(app_main_sources)}")
    print("  runtime_control_queue_mechanics=isolated")
    print("  runtime_control_transport_io=isolated")


if __name__ == "__main__":
    main()
