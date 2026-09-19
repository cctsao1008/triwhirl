from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class ToolCommand:
    group: str
    name: str
    script: str
    description: str


COMMANDS: tuple[ToolCommand, ...] = (
    ToolCommand(
        "log",
        "capture-uart",
        "logging/capture.py",
        "Capture live UART telemetry to schema-versioned CSV.",
    ),
    ToolCommand(
        "log",
        "download",
        "parameter_id/download_log_ble.py",
        "Download a completed TWLG binary log over BLE.",
    ),
    ToolCommand(
        "log",
        "decode",
        "parameter_id/decode_twlog.py",
        "Validate and decode a TWLG binary log to CSV.",
    ),
    ToolCommand(
        "id",
        "actuator-uart",
        "parameter_id/acquire.py",
        "Run tethered UART actuator identification acquisition.",
    ),
    ToolCommand(
        "id",
        "actuator-ble",
        "parameter_id/acquire_ble.py",
        "Run untethered BLE actuator identification acquisition.",
    ),
    ToolCommand(
        "id",
        "body-free",
        "parameter_id/body_free_ble.py",
        "Acquire untethered free-body motion over BLE.",
    ),
    ToolCommand(
        "id",
        "body-local",
        "parameter_id/body_local_ble.py",
        "Acquire passive local upright release data over BLE.",
    ),
    ToolCommand(
        "id",
        "body-active",
        "parameter_id/body_active_ble.py",
        "Acquire active local upright identification data over BLE.",
    ),
    ToolCommand(
        "id",
        "swing",
        "parameter_id/auto_swing_id_ble.py",
        "Run autonomous reaction-wheel swing identification acquisition.",
    ),
    ToolCommand(
        "fit",
        "actuator",
        "parameter_id/local_fit.py",
        "Fit the preliminary actuator/local continuous-time model.",
    ),
    ToolCommand(
        "fit",
        "body-local",
        "parameter_id/body_local_fit.py",
        "Fit the passive local upright body model.",
    ),
    ToolCommand(
        "fit",
        "body-active",
        "parameter_id/body_active_fit.py",
        "Fit active A/B/C upright vertex models.",
    ),
)


def groups() -> tuple[str, ...]:
    return tuple(dict.fromkeys(command.group for command in COMMANDS))


def commands_for_group(group: str) -> tuple[ToolCommand, ...]:
    return tuple(command for command in COMMANDS if command.group == group)


def find_command(group: str, name: str) -> ToolCommand | None:
    for command in COMMANDS:
        if command.group == group and command.name == name:
            return command
    return None


def resolve_script(tools_root: Path, command: ToolCommand) -> Path:
    return tools_root / command.script


def iter_commands() -> Iterable[ToolCommand]:
    return iter(COMMANDS)
