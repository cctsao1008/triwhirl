#!/usr/bin/env python3
"""Fail CI if runtime composition debt grows during issue #32/A2."""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MAIN = ROOT / "main"
TESTS = ROOT / "tools" / "tests"

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

SUPERVISOR_REQUIRED_READ_ONLY_COMMANDS = (
    'std::strcmp(line, "attitude status")',
    'std::strcmp(line, "fault status")',
    'std::strcmp(line, "ble status")',
    "readLatestRuntimeSnapshot(",
    "triwhirl::ble::connected()",
    "triwhirl::ble::subscribed()",
)

PARSER_REQUIRED_TYPED_COMMANDS = (
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
)

SUPERVISOR_REQUIRED_TYPED_BOUNDARY = (
    "parseRuntimeCommand(",
    "RuntimeCommandParseStatus::kCommand",
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

PARSER_TEST_REQUIRED_TOKENS = (
    'expectType("motor stop", RuntimeCommandType::kMotorStop)',
    'parseRuntimeCommand("motor vq -0.625")',
    'parseRuntimeCommand("field 3.5 0.8")',
    'parseRuntimeCommand("attitude reset -1.25")',
    'parseRuntimeCommand("imu calibrate 750")',
    'RuntimeCommandParseStatus::kUsageError',
    'RuntimeCommandParseStatus::kNotMatched',
)

CPP_INCLUDE_RE = re.compile(r'^\s*#\s*include\s+"([^"]+\.cpp)"', re.MULTILINE)
RENAME_DEFINE_RE = re.compile(
    r"^\s*#\s*define\s+(app_main|updateEncoder|updateImu|initEncoderBus|initImuBus)\s+(\w+)",
    re.MULTILINE,
)
APP_MAIN_RE = re.compile(r'extern\s+"C"\s+void\s+app_main\s*\(')


def fail(message: str) -> None:
    raise SystemExit(f"runtime architecture contract: {message}")


def require_tokens(text: str, tokens: tuple[str, ...], message: str) -> None:
    missing = [token for token in tokens if token not in text]
    if missing:
        fail(f"{message}: {missing}")


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
        fail(f"queue mechanics leaked into runtime_control.cpp: {leaked_queue_mechanics}")

    leaked_supervisor_io = [
        token for token in CONTROL_FORBIDDEN_SUPERVISOR_IO if token in control_text
    ]
    if leaked_supervisor_io:
        fail(f"UART development/BLE GATT ingress leaked into runtime_control.cpp: {leaked_supervisor_io}")

    supervisor_text = (MAIN / "runtime_supervisor_io.cpp").read_text(encoding="utf-8")
    parser_text = (MAIN / "runtime_command_parser.cpp").read_text(encoding="utf-8")
    parser_test = TESTS / "runtime_command_parser_test.cpp"
    if not parser_test.exists():
        fail("runtime command parser contract test is missing")
    parser_test_text = parser_test.read_text(encoding="utf-8")

    require_tokens(supervisor_text, SUPERVISOR_REQUIRED_READ_ONLY_COMMANDS,
                   "Core0 read-only diagnostics regressed")
    require_tokens(supervisor_text, SUPERVISOR_REQUIRED_TYPED_BOUNDARY,
                   "supervisor typed-command boundary regressed")
    require_tokens(parser_text, PARSER_REQUIRED_TYPED_COMMANDS,
                   "Core0 typed command parser regressed")
    require_tokens(control_text, CONTROL_REQUIRED_TYPED_COMMANDS,
                   "typed realtime command execution regressed")
    require_tokens(parser_test_text, PARSER_TEST_REQUIRED_TOKENS,
                   "runtime command parser contract coverage regressed")

    print("runtime architecture contract: PASS")
    print(f"  cpp_includes={sorted(cpp_includes)}")
    print(f"  renaming_shims={sorted(renaming_defines)}")
    print(f"  app_main_sources={sorted(app_main_sources)}")
    print("  runtime_control_queue_mechanics=isolated")
    print("  runtime_control_uart_dev_ble_gatt_ingress=absent")
    print("  supervisor_read_only=attitude,fault,ble")
    print("  supervisor_transports=uart_dev,ble_gatt")
    print("  command_parser=explicit_core0_service")
    print("  command_parser_contract_test=present")
    print("  typed_runtime_commands=motor_stop,stop,swing_abort,timing_reset,fault_clear,telemetry_on,telemetry_off,motor_vq,field,attitude_reset,imu_calibrate")


if __name__ == "__main__":
    main()
