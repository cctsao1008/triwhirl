from __future__ import annotations

import importlib
import subprocess
import sys
from pathlib import Path
from typing import Callable, Sequence

from . import __version__
from .registry import commands_for_group, find_command, groups, iter_commands, resolve_script


TOOLS_ROOT = Path(__file__).resolve().parents[1]
NativeHandler = Callable[[Sequence[str]], int]


def _print_usage() -> None:
    print("TriWhirl Toolbox (twtool)")
    print()
    print("Usage:")
    print("  python tools/twtool.py <group> <command> [command arguments...]")
    print("  python -m tools.triwhirl_tool <group> <command> [command arguments...]")
    print("  python tools/twtool.py help <group> <command>")
    print("  python tools/twtool.py --list")
    print()
    print("Groups:")
    for group in groups():
        print(f"  {group}")
        for command in commands_for_group(group):
            backend = "native" if command.handler else "legacy"
            print(f"    {command.name:<14} [{backend}] {command.description}")
    print()
    print("Legacy scripts remain supported during the toolbox migration.")


def _print_compact_list() -> None:
    for command in iter_commands():
        target = command.handler if command.handler else command.script
        backend = "native" if command.handler else "legacy"
        print(f"{command.group} {command.name}\t{backend}\t{target}")


def _load_handler(spec: str) -> NativeHandler:
    module_name, separator, attribute = spec.partition(":")
    if not separator or not module_name or not attribute:
        raise RuntimeError(f"invalid toolbox handler specification: {spec}")
    module = importlib.import_module(module_name)
    handler = getattr(module, attribute, None)
    if handler is None or not callable(handler):
        raise RuntimeError(f"toolbox handler not found: {spec}")
    return handler


def _dispatch(group: str, name: str, args: Sequence[str]) -> int:
    command = find_command(group, name)
    if command is None:
        print(f"twtool: unknown command: {group} {name}", file=sys.stderr)
        available = commands_for_group(group)
        if available:
            print(
                "available: " + ", ".join(item.name for item in available),
                file=sys.stderr,
            )
        else:
            print("groups: " + ", ".join(groups()), file=sys.stderr)
        return 2

    if command.handler is not None:
        try:
            handler = _load_handler(command.handler)
            return int(handler(args))
        except (ImportError, RuntimeError, AttributeError) as exc:
            print(f"twtool: {exc}", file=sys.stderr)
            return 2

    script = resolve_script(TOOLS_ROOT, command)
    if not script.is_file():
        print(
            f"twtool: registered script is missing: {script.relative_to(TOOLS_ROOT)}",
            file=sys.stderr,
        )
        return 2

    completed = subprocess.run([sys.executable, str(script), *args], check=False)
    return int(completed.returncode)


def main(argv: Sequence[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if not args or args[0] in {"-h", "--help"}:
        _print_usage()
        return 0
    if args[0] == "--version":
        print(__version__)
        return 0
    if args[0] == "--list":
        _print_compact_list()
        return 0

    if args[0] == "help":
        if len(args) == 1:
            _print_usage()
            return 0
        if len(args) != 3:
            print("twtool: usage: help <group> <command>", file=sys.stderr)
            return 2
        return _dispatch(args[1], args[2], ["--help"])

    if len(args) < 2:
        print(f"twtool: missing command after group '{args[0]}'", file=sys.stderr)
        available = commands_for_group(args[0])
        if available:
            print(
                "available: " + ", ".join(item.name for item in available),
                file=sys.stderr,
            )
        else:
            print("groups: " + ", ".join(groups()), file=sys.stderr)
        return 2

    return _dispatch(args[0], args[1], args[2:])


if __name__ == "__main__":
    raise SystemExit(main())
