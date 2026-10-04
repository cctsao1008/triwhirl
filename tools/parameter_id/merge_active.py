#!/usr/bin/env python3
"""Merge independent TriWhirl body-active acquisitions without trial-ID collisions.

The current active-ID coordinate is vertex agnostic. `theta_ref_rad` is retained
as provenance for every source/trial; `vertex_id` is accepted when present in a
historical CSV but is never required for merging.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Merge body-active CSV runs into one fit-ready dataset while renumbering "
            "trial IDs globally and preserving per-trial theta_ref provenance."
        )
    )
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser.parse_args()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_theta_refs(rows: list[dict[str, str]]) -> list[float]:
    values: list[float] = []
    for row in rows:
        text = row.get("theta_ref_rad", "").strip()
        if not text:
            continue
        try:
            value = float(text)
        except ValueError:
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def main() -> int:
    args = parse_args()
    if len(args.inputs) < 1:
        raise RuntimeError("at least one input CSV is required")

    fieldnames: list[str] | None = None
    merged: list[dict[str, str]] = []
    next_trial = 1
    sources: list[dict[str, object]] = []

    for source in args.inputs:
        with source.open("r", newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            current_fields = list(reader.fieldnames or [])
            if not current_fields:
                raise RuntimeError(f"{source}: missing CSV header")
            if "trial" not in current_fields or "theta_ref_rad" not in current_fields:
                raise RuntimeError(
                    f"{source}: not a body-active local CSV; require trial and theta_ref_rad"
                )
            if fieldnames is None:
                fieldnames = current_fields
            elif current_fields != fieldnames:
                raise RuntimeError(
                    f"{source}: schema differs from the first input; do not silently merge"
                )
            rows = list(reader)

        source_trials = sorted({int(row["trial"]) for row in rows if row.get("trial")})
        if not source_trials:
            raise RuntimeError(f"{source}: contains no trials")
        refs = finite_theta_refs(rows)
        if not refs:
            raise RuntimeError(f"{source}: contains no finite theta_ref_rad values")

        trial_map = {old: next_trial + i for i, old in enumerate(source_trials)}
        legacy_vertices = sorted(
            {
                row.get("vertex_id", "").strip()
                for row in rows
                if row.get("vertex_id", "").strip()
            }
        )
        for row in rows:
            if not row.get("trial"):
                continue
            rewritten = dict(row)
            rewritten["trial"] = str(trial_map[int(row["trial"])])
            merged.append(rewritten)
        next_trial += len(source_trials)
        sources.append(
            {
                "path": str(source),
                "sha256": sha256(source),
                "rows": len(rows),
                "source_trials": source_trials,
                "merged_trial_map": {str(key): value for key, value in trial_map.items()},
                "theta_ref_min_rad": min(refs),
                "theta_ref_max_rad": max(refs),
                "legacy_vertices": legacy_vertices,
            }
        )

    assert fieldnames is not None
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(merged)

    merged_refs = finite_theta_refs(merged)
    legacy_vertices = sorted(
        {
            row.get("vertex_id", "").strip()
            for row in merged
            if row.get("vertex_id", "").strip()
        }
    )
    manifest = {
        "format": "triwhirl-active-merge-v2",
        "coordinate_contract": (
            "theta_ref_rad is per-trial provenance; no A/B/C label is required"
        ),
        "output": str(args.output),
        "output_sha256": sha256(args.output),
        "rows": len(merged),
        "trials": next_trial - 1,
        "theta_ref_min_rad": min(merged_refs),
        "theta_ref_max_rad": max(merged_refs),
        "legacy_vertices": legacy_vertices,
        "sources": sources,
    }
    manifest_path = args.output.with_suffix(args.output.suffix + ".merge.json")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    print(
        "active_merge,"
        f"inputs={len(args.inputs)},rows={len(merged)},trials={next_trial - 1},"
        f"theta_ref_min_rad={manifest['theta_ref_min_rad']:.6f},"
        f"theta_ref_max_rad={manifest['theta_ref_max_rad']:.6f}"
    )
    print(f"active_merge_csv={args.output}")
    print(f"active_merge_manifest={manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
