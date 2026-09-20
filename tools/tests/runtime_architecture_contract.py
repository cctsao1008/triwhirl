#!/usr/bin/env python3
"""Fail CI if runtime composition debt regresses during issue #32/A2."""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MAIN = ROOT / "main"
TESTS = ROOT / "tools" / "tests"

CONTROL_FORBIDDEN_SUPERVISOR_IO = ("uart_read_bytes(", "triwhirl::ble::read(")

SUPERVISOR_REQUIRED_READ_ONLY_COMMANDS = (
    'std::strcmp(line, "attitude status")',
    'std::strcmp(line, "fault status")',
    'std::strcmp(line, "ble status")',
    'std::strcmp(line, "timing status")',
    'std::strcmp(line, "telemetry")',
    'std::strcmp(line, "help")',
    "readLatestRuntimeSnapshot(",
    "triwhirl::ble::connected()",
    "triwhirl::ble::subscribed()",
)

REQUIRED_TYPES = (
    "kStatus", "kMotorStatus", "kImuStatus", "kLogStatus", "kSwingStatus",
    "kTimingProfileStatus", "kMotorStop", "kStop", "kSwingAbort", "kSwingStart",
    "kSwingConfig", "kTimingReset", "kTimingProfileOn", "kTimingProfileOff",
    "kTimingProfileReset", "kFaultClear", "kTelemetryOn", "kTelemetryOff",
    "kMotorVq", "kMotorConfig", "kMotorCalibrate", "kField", "kAttitudeReset",
    "kImuCalibrate", "kImuMap", "kLogPrepare", "kLogStart", "kLogCriticalOn",
    "kLogCriticalOff", "kLogStop", "kLogDump",
)

PARSER_TEST_REQUIRED_TOKENS = (
    'expectType("status", RuntimeCommandType::kStatus)',
    'expectType("swing status", RuntimeCommandType::kSwingStatus)',
    'expectType("timing profile status", RuntimeCommandType::kTimingProfileStatus)',
    'expectType("log dump", RuntimeCommandType::kLogDump)',
    'parseRuntimeCommand("log prepare 12.5")',
    'parseRuntimeCommand("motor vq -0.625")',
    'parseRuntimeCommand("imu map 0 1 2 1 -1 1")',
    'parseRuntimeCommand(\n        "swing config 24 0.4 0.8 5 9 14 12.5 0.2 -1 68 20")',
    'parseRuntimeCommand("definitely unknown")',
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
        cpp_includes.update((path.name, x) for x in CPP_INCLUDE_RE.findall(text))
        renaming_defines.update((path.name, a, b) for a, b in RENAME_DEFINE_RE.findall(text))
        if APP_MAIN_RE.search(text):
            app_main_sources.append(path.name)

    if cpp_includes:
        fail(f".cpp source inclusion is forbidden: {sorted(cpp_includes)}")
    if renaming_defines:
        fail(f"symbol-renaming shims are forbidden: {sorted(renaming_defines)}")
    if (MAIN / "runtime_control.cpp").exists():
        fail("obsolete runtime_control.cpp wrapper returned")
    if (MAIN / "app_main.cpp").exists():
        fail("obsolete legacy app_main.cpp returned")
    if app_main_sources != ["runtime_main.cpp"]:
        fail(f"application entry ownership regressed: {sorted(app_main_sources)}")

    runtime_main_text = (MAIN / "runtime_main.cpp").read_text(encoding="utf-8")
    runtime_state_header = (MAIN / "runtime_state.hpp").read_text(encoding="utf-8")
    runtime_state_text = (MAIN / "runtime_state.cpp").read_text(encoding="utf-8")
    cmake_text = (MAIN / "CMakeLists.txt").read_text(encoding="utf-8")
    supervisor_text = (MAIN / "runtime_supervisor_io.cpp").read_text(encoding="utf-8")
    supervisor_header = (MAIN / "runtime_supervisor_io.hpp").read_text(encoding="utf-8")
    command_header = (MAIN / "runtime_command.hpp").read_text(encoding="utf-8")
    parser_text = (MAIN / "runtime_command_parser.cpp").read_text(encoding="utf-8")
    parser_test_text = (TESTS / "runtime_command_parser_test.cpp").read_text(encoding="utf-8")

    leaked_io = [x for x in CONTROL_FORBIDDEN_SUPERVISOR_IO if x in runtime_main_text]
    if leaked_io:
        fail(f"UART development/BLE GATT ingress leaked into realtime runtime: {leaked_io}")

    if "SupervisorInputEventType::kCommand" in runtime_main_text or \
       "SupervisorInputEventType::kCommand" in supervisor_text or \
       "SupervisorInputEventType::kCommand" in supervisor_header:
        fail("legacy raw command event remains")
    if "handleSupervisorCommand(event.line)" in runtime_main_text:
        fail("realtime still dispatches raw command strings")
    if "char line[kSupervisorCommandBytes]" in supervisor_header:
        fail("SupervisorInputEvent still carries a raw command line")

    obsolete_runtime_bridge = (
        "handleSupervisorCommand(", "consumeSupervisorBytes(",
        "pollSupervisorConsole(", "handleSwingCommand(",
        "parseSwingConfig(", "handleRuntimeTimingProfileCommand(",
        "commandAllowedDuringSwing(", "triwhirl_legacy_updateEncoder",
        "triwhirl_legacy_updateImu", "triwhirl_legacy_initEncoderBus",
        "triwhirl_legacy_initImuBus", "triwhirl_legacy_app_main",
    )
    leaked_bridge = [x for x in obsolete_runtime_bridge if x in runtime_main_text]
    if leaked_bridge:
        fail(f"obsolete runtime bridge remains: {leaked_bridge}")

    require_tokens(runtime_main_text,
                   ("initRuntimeEncoderBus(", "initRuntimeImuBus(",
                    "void triwhirl::runtime::realtimeControlTask(",
                    "initEncoderAcquisition(", "waitForNextRealtimeRelease(",
                    '#include "runtime_state.hpp"',
                    'extern "C" void app_main(void)'),
                   "explicit realtime/startup ownership regressed")

    require_tokens(runtime_state_header,
                   ("namespace triwhirl::runtime::state", "extern As5600 encoder",
                    "extern Mpu6050 imu", "extern SafetyLatch safety_latch",
                    "extern RuntimeLogger runtime_logger", "void updateMotor(",
                    "bool sampleImu(", "void consoleWriteBytes("),
                   "runtime state contract is incomplete")
    require_tokens(runtime_state_text,
                   ("As5600 encoder;", "Mpu6050 imu;", "void updateMotor(",
                    "void evaluateSafety(", "void emitTelemetry(",
                    "bool initConsole("),
                   "runtime state implementation is incomplete")
    if '"runtime_state.cpp"' not in cmake_text:
        fail("runtime_state.cpp is not compiled explicitly")

    first_snapshot = runtime_main_text.find("publishSupervisorSnapshot(")
    supervisor_init = runtime_main_text.find("initSupervisorIo(")
    if first_snapshot < 0 or supervisor_init < 0 or first_snapshot > supervisor_init:
        fail("initial runtime snapshot must be published before supervisor ingress starts")

    require_tokens(supervisor_text, SUPERVISOR_REQUIRED_READ_ONLY_COMMANDS,
                   "Core0 read-only diagnostics regressed")
    require_tokens(supervisor_text,
                   ("parseRuntimeCommand(", "SupervisorInputEventType::kRuntimeCommand",
                    "ERR unknown command", "uart_development_input", "ble_gatt_input"),
                   "supervisor typed-command boundary regressed")
    require_tokens(command_header, REQUIRED_TYPES,
                   "typed command contract is incomplete")
    require_tokens(parser_text, tuple(f"RuntimeCommandType::{x}" for x in REQUIRED_TYPES),
                   "Core0 typed command parser is incomplete")
    require_tokens(runtime_main_text, tuple(f"RuntimeCommandType::{x}" for x in REQUIRED_TYPES),
                   "typed realtime command execution is incomplete")
    require_tokens(parser_test_text, PARSER_TEST_REQUIRED_TOKENS,
                   "runtime command parser contract coverage regressed")

    print("runtime architecture contract: PASS")
    print("  cpp_includes=[]")
    print("  renaming_shims=[]")
    print("  app_main_sources=['runtime_main.cpp']")
    print("  runtime_control_wrapper=absent")
    print("  legacy_app_main=absent")
    print("  runtime_state_service=explicit_compiled")
    print("  raw_command_strings_cross_realtime=no")
    print("  obsolete_runtime_bridge=absent")
    print("  runtime_i2c_startup=explicit_named_helpers")
    print("  runtime_uart_dev_ble_gatt_ingress=absent")
    print("  supervisor_read_only=attitude,fault,ble,timing,telemetry,help")
    print("  supervisor_snapshot_ready_before_ingress=yes")
    print("  supervisor_transports=uart_dev,ble_gatt")
    print("  command_parser=complete_core0_grammar")
    print("  command_parser_contract_test=present")


if __name__ == "__main__":
    main()
