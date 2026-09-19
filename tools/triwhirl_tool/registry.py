from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class ToolCommand:
    group: str
    name: str
    description: str
    script: str | None = None
    handler: str | None = None

    def __post_init__(self) -> None:
        if (self.script is None) == (self.handler is None):
            raise ValueError(
                f"command {self.group} {self.name} must define exactly one backend"
            )


COMMANDS: tuple[ToolCommand, ...] = (
    ToolCommand(
        group="log",
        name="status",
        handler="commands.log:status_main",
        description="Read firmware TWLG logger state over BLE.",
    ),
    ToolCommand(
        group="log",
        name="prepare",
        handler="commands.log:prepare_main",
        description="Pre-erase/prepare the TWLG region before a realtime run.",
    ),
    ToolCommand(
        group="log",
        name="start",
        handler="commands.log:start_main",
        description="Start synchronized 1 kHz firmware TWLG capture.",
    ),
    ToolCommand(
        group="log",
        name="critical",
        handler="commands.log:critical_main",
        description="Pause/resume flash programming while SRAM capture continues.",
    ),
    ToolCommand(
        group="log",
        name="stop",
        handler="commands.log:stop_main",
        description="Stop TWLG capture and wait for flash finalization.",
    ),
    ToolCommand(
        group="log",
        name="session",
        handler="commands.session:session_main",
        description="Prepare, record, finalize, download, and optionally decode TWLG.",
    ),
    ToolCommand(
        group="log",
        name="capture-uart",
        script="logging/capture.py",
        description="Capture live UART telemetry to schema-versioned CSV.",
    ),
    ToolCommand(
        group="log",
        name="download",
        handler="commands.log:download_main",
        description="Download and validate a completed TWLG binary log over BLE.",
    ),
    ToolCommand(
        group="log",
        name="decode",
        handler="commands.log:decode_main",
        description="Validate and decode a TWLG binary log to CSV.",
    ),
    ToolCommand(
        group="log",
        name="inspect",
        handler="commands.log:inspect_main",
        description="Inspect TWLG metadata and signal ranges without converting it.",
    ),
    ToolCommand(
        group="id",
        name="actuator-uart",
        script="parameter_id/acquire.py",
        description="Run tethered UART actuator identification acquisition.",
    ),
    ToolCommand(
        group="id",
        name="actuator-ble",
        script="parameter_id/acquire_ble.py",
        description="Run untethered BLE actuator identification acquisition.",
    ),
    ToolCommand(
        group="id",
        name="body-free",
        script="parameter_id/body_free_ble.py",
        description="Acquire untethered free-body motion over BLE.",
    ),
    ToolCommand(
        group="id",
        name="body-local",
        script="parameter_id/body_local_ble.py",
        description="Acquire passive local upright release data over BLE.",
    ),
    ToolCommand(
        group="id",
        name="body-active",
        script="parameter_id/body_active_ble.py",
        description="Acquire active local upright identification data over BLE.",
    ),
    ToolCommand(
        group="id",
        name="swing",
        script="parameter_id/auto_swing_id_ble.py",
        description="Run autonomous reaction-wheel swing identification acquisition.",
    ),
    ToolCommand(
        group="fit",
        name="actuator",
        script="parameter_id/local_fit.py",
        description="Fit the preliminary actuator/local continuous-time model.",
    ),
    ToolCommand(
        group="fit",
        name="body-local",
        script="parameter_id/body_local_fit.py",
        description="Fit the passive local upright body model.",
    ),
    ToolCommand(
        group="fit",
        name="body-active",
        script="parameter_id/body_active_fit.py",
        description="Fit active A/B/C upright vertex models.",
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
    if command.script is None:
        raise ValueError(f"command {command.group} {command.name} is not script-backed")
    return tools_root / command.script


def iter_commands() -> Iterable[ToolCommand]:
    return iter(COMMANDS)
