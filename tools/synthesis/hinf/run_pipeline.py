#!/usr/bin/env python3
"""Run fit -> linearization -> uncertainty -> H-infinity synthesis in one command."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a vertex-normalized robust plant and synthesize one H-infinity gain."
    )
    parser.add_argument("input", type=Path, help="fit-ready *-active.csv")
    parser.add_argument("--vertices", default="B,C")
    parser.add_argument("--nominal", default="mean")
    parser.add_argument("--derivative-window", type=int, default=3)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/hinf"))
    parser.add_argument("--provenance-note", default="")
    parser.add_argument("--solver", default="CLARABEL")
    return parser.parse_args()


def run(command: list[str]) -> None:
    print("+ " + " ".join(command))
    subprocess.run(command, check=True)


def main() -> int:
    args = parse_args()
    repo = Path(__file__).resolve().parents[3]
    stem = args.input.stem.removesuffix("-active")
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)

    fit = out / f"{stem}-fit.json"
    linear = out / f"{stem}-linear.json"
    poly = out / f"{stem}-polytope.json"
    controller = out / f"{stem}-hinf.json"

    run(
        [
            sys.executable,
            str(repo / "tools/parameter_id/body_active_fit.py"),
            str(args.input),
            "--derivative-window",
            str(args.derivative_window),
            "-o",
            str(fit),
        ]
    )
    run(
        [
            sys.executable,
            str(repo / "model/linearization/from_active_fit.py"),
            str(fit),
            "--vertices",
            args.vertices,
            "--nominal",
            args.nominal,
            "--provenance-note",
            args.provenance_note,
            "-o",
            str(linear),
        ]
    )
    run(
        [
            sys.executable,
            str(repo / "model/uncertainty/build_polytopic.py"),
            str(linear),
            "-o",
            str(poly),
        ]
    )
    run(
        [
            sys.executable,
            str(repo / "tools/synthesis/hinf/synthesize.py"),
            str(poly),
            "--solver",
            args.solver,
            "-o",
            str(controller),
        ]
    )

    print("pipeline complete")
    print(f"fit:        {fit}")
    print(f"linear:     {linear}")
    print(f"polytope:   {poly}")
    print(f"controller: {controller}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
