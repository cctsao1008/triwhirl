#!/usr/bin/env python3
"""Merge independent TriWhirl body-active acquisitions without trial-ID collisions."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Merge body-active CSV runs into one fit-ready dataset while renumbering "
            "trial IDs globally. Intended for separate A/B/C acquisitions."
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
            if "trial" not in current_fields or "vertex_id" not in current_fields:
                raise RuntimeError(f"{source}: not a body-active CSV")
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
        trial_map = {old: next_trial + i for i, old in enumerate(source_trials)}
        vertices = sorted({row.get("vertex_id", "").strip() for row in rows if row.get("vertex_id", "").strip()})
        for row in rows:
            if not row.get("trial"):
                continue
            row = dict(row)
            row["trial"] = str(trial_map[int(row["trial"])])
            merged.append(row)
        next_trial += len(source_trials)
        sources.append(
            {
                "path": str(source),
                "sha256": sha256(source),
                "rows": len(rows),
                "source_trials": source_trials,
                "merged_trial_map": {str(k): v for k, v in trial_map.items()},
                "vertices": vertices,
            }
        )

    assert fieldnames is not None
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(merged)

    manifest = {
        "format": "triwhirl-active-merge-v1",
        "output": str(args.output),
        "output_sha256": sha256(args.output),
        "rows": len(merged),
        "trials": next_trial - 1,
        "vertices": sorted(
            {row.get("vertex_id", "").strip() for row in merged if row.get("vertex_id", "").strip()}
        ),
        "sources": sources,
    }
    manifest_path = args.output.with_suffix(args.output.suffix + ".merge.json")
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    print(
        "active_merge,"
        f"inputs={len(args.inputs)},rows={len(merged)},trials={next_trial - 1},"
        f"vertices={','.join(manifest['vertices'])}"
    )
    print(f"active_merge_csv={args.output}")
    print(f"active_merge_manifest={manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
