from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from ..standup_plot import plot_standup_trace


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plot a TriWhirl standup .twtrace or decoded standup CSV."
    )
    parser.add_argument("input", type=Path)
    parser.add_argument("-o", "--output", type=Path, default=None)
    parser.add_argument("--show", action="store_true", help="also open an interactive plot window")
    parser.add_argument("--title", default=None)
    return parser


def standup_plot_main(argv: Sequence[str]) -> int:
    args = _parser().parse_args(list(argv))
    try:
        output = plot_standup_trace(
            args.input,
            args.output,
            show=args.show,
            title=args.title,
        )
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}")
        return 1
    print(f"standup_plot={output}")
    return 0
