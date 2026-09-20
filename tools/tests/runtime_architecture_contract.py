#!/usr/bin/env python3
"""Fail CI if runtime composition debt grows during issue #32/A2.

The active firmware still has one explicitly tolerated source-inclusion chain:

    runtime_control.cpp -> runtime_main.cpp -> app_main.cpp

and five symbol-renaming shims in runtime_main.cpp. A2 is removing those pieces
incrementally. Until they are gone, CI prevents new .cpp includes, new
symbol-renaming interception, a second application-entry owner, queue/task
mechanics from leaking back into realtime control, UART development ingress or
BLE GATT payload draining from returning to Core 1, migrated read-only
diagnostics from falling back to live realtime formatting, or migrated mutating
commands from regressing to strings.
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

CONTROL_FORBIDDEN_SUPERVISOR_IO = (
    "uart_read_bytes(",
    "triwhirl::ble::read(",
)

SUPERVISOR_REQUIRED_SNAPSHOT_COMMANDS = (
    'std::strcmp(line, "attitude status")',
    'std::strcmp(line, "fault status")',
    "readLatestRuntimeSnapshot(",
)

SUPERVISOR_REQUIRED_TYPED_COMMANDS = (
    'std::strcmp(line, "motor stop")',
    'std::strcmp(line, "stop")',
    'std::strcmp(line, "swing abort")',
    'std::strcmp(line, "timing reset")',
    'std::strcmp(line, "fault clear")',
    'std::strcmp(line, "telemetry on")',
    'std::strcmp(line, "telemetry off")',
    'commandArguments(line, "motor vq"',
    'commandArguments(line, "field"',
    'commandArguments(line, "attitude reset"',
    'commandArguments(line, "imu calibrate"',
    "RuntimeCommandType::kMotorStop",
    "RuntimeCommandType::kStop",
    "RuntimeCommandType::kSwingAbort",
    "RuntimeCommandType::kTimingReset",
    "RuntimeCommandType::kFaultClear",
    "RuntimeCommandType::kTelemetryOn",
    "RuntimeCommandType::kTelemetryOff",
    "RuntimeCommandType::kMotorVq",
    "RuntimeCommandType::kField",
    "RuntimeCommandType::kAttitudeReset",
    "RuntimeCommandType::kImuCalibrate",
    "SupervisorInputEventType::kRuntimeCommand",
    "uart_development_input",
    "ble_gatt_input",
)

CONTROL_REQUIRED_TYPED_COMMANDS = (
    "executeRuntimeCommand(",
    "typedCommandAllowedDuringSwing(",
    "RuntimeCommandType::kMotorStop",
    "RuntimeCommandType::kStop",
    "RuntimeCommandType::kSwingAbort",
    "RuntimeCommandType::kTimingReset",
    "RuntimeCommandType::kFaultClear",
    "RuntimeCommandType::kTelemetryOn",
    "RuntimeCommandType::kTelemetryOff",
    "RuntimeCommandType::kMotorVq",
    "RuntimeCommandType::kField",
    "RuntimeCommandType::kAttitudeReset",
    "RuntimeCommandType::kImuCalibrate",
)

CPP_INCLUDE_RE = re.compile(r'^\s*#\s*include\s+"([^"]+\.cpp)"', re.MULTILINE)
RENAME_DEFINE_RE = re.compile(
    r"^\s*#\s*define\s+(app_main|updateEncoder|updateImu|initEncoderBus|initImuBus)\s+(\w+)",
    re.MULTILINE,
)
APP_MAIN_RE = re.compile(r'extern\s+"C"\s+void\s+app_main\s*\(')


def fail(message: str) -> None:
    raise SystemExit(f"runtime architecture contract: {message}")


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

    unexpected_entries = set(app_main_sources) - {"app_main.cpp", "runtime_main.cpp"}
    if unexpected_entries:
        fail(f"unexpected app_main owner(s): {sorted(unexpected_entries)}")

    control_text = (MAIN / "runtime_control.cpp").read_text(encoding="utf-8")
    leaked_queue_mechanics = [
        token for token in CONTROL_FORBIDDEN_QUEUE_MECHANICS if token in control_text
    ]
    if leaked_queue_mechanics:
        fail(
            "encoder/supervisor queue mechanics leaked into runtime_control.cpp: "
            f"{leaked_queue_mechanics}"
        )

    leaked_supervisor_io = [
        token for token in CONTROL_FORBIDDEN_SUPERVISOR_IO if token in control_text
    ]
    if leaked_supervisor_io:
        fail(
            "UART development/BLE GATT ingress leaked into runtime_control.cpp: "
            f"{leaked_supervisor_io}"
        )

    supervisor_text = (MAIN / "runtime_supervisor_io.cpp").read_text(encoding="utf-8")
    missing_snapshot_commands = [
        token
        for token in SUPERVISOR_REQUIRED_SNAPSHOT_COMMANDS
        if token not in supervisor_text
    ]
    if missing_snapshot_commands:
        fail(
            "snapshot-backed read-only diagnostics regressed: "
            f"{missing_snapshot_commands}"
        )

    missing_typed_supervisor = [
        token for token in SUPERVISOR_REQUIRED_TYPED_COMMANDS if token not in supervisor_text
    ]
    if missing_typed_supervisor:
        fail(
            "typed supervisor command parsing regressed: "
            f"{missing_typed_supervisor}"
        )

    missing_typed_control = [
        token for token in CONTROL_REQUIRED_TYPED_COMMANDS if token not in control_text
    ]
    if missing_typed_control:
        fail(
            "typed realtime command execution regressed: "
            f"{missing_typed_control}"
        )

    print("runtime architecture contract: PASS")
    print(f"  cpp_includes={sorted(cpp_includes)}")
    print(f"  renaming_shims={sorted(renaming_defines)}")
    print(f"  app_main_sources={sorted(app_main_sources)}")
    print("  runtime_control_queue_mechanics=isolated")
    print("  runtime_control_uart_dev_ble_gatt_ingress=absent")
    print("  supervisor_snapshot_diagnostics=attitude,fault")
    print("  supervisor_transports=uart_dev,ble_gatt")
    print("  typed_runtime_commands=motor_stop,stop,swing_abort,timing_reset,fault_clear,telemetry_on,telemetry_off,motor_vq,field,attitude_reset,imu_calibrate")


if __name__ == "__main__":
    main()
