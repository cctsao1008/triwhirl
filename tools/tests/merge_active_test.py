#!/usr/bin/env python3

from __future__ import annotations

import csv
import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def write_run(path: Path, vertex: str, theta_ref: float) -> None:
    fields = [
        "schema_version",
        "trial",
        "vertex_id",
        "vertex_center_deg",
        "phase",
        "theta_ref_rad",
        "planned_vq_v",
        "t_us",
        "theta_rad",
        "theta_rate_rad_s",
        "vel_rad_s",
        "vq_v",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for trial in (1, 2):
            for sample in range(3):
                writer.writerow(
                    {
                        "schema_version": "1",
                        "trial": trial,
                        "vertex_id": vertex,
                        "vertex_center_deg": "68.0",
                        "phase": "active",
                        "theta_ref_rad": theta_ref,
                        "planned_vq_v": "0.25" if trial == 1 else "-0.25",
                        "t_us": trial * 10000 + sample * 1000,
                        "theta_rad": theta_ref,
                        "theta_rate_rad_s": "0.1",
                        "vel_rad_s": "0.2",
                        "vq_v": "0.25" if trial == 1 else "-0.25",
                    }
                )


def main() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        a = root / "a.csv"
        b = root / "b.csv"
        merged = root / "merged.csv"
        write_run(a, "A", 1.18)
        write_run(b, "B", -0.91)
        subprocess.run(
            [
                sys.executable,
                str(ROOT / "tools/parameter_id/merge_active.py"),
                str(a),
                str(b),
                "-o",
                str(merged),
            ],
            check=True,
        )

        rows = list(csv.DictReader(merged.open(encoding="utf-8")))
        assert sorted({int(row["trial"]) for row in rows}) == [1, 2, 3, 4]
        assert sorted({row["vertex_id"] for row in rows}) == ["A", "B"]

        manifest = json.loads(
            merged.with_suffix(merged.suffix + ".merge.json").read_text(encoding="utf-8")
        )
        assert manifest["format"] == "triwhirl-active-merge-v1"
        assert manifest["trials"] == 4
        assert manifest["vertices"] == ["A", "B"]
        assert len(manifest["sources"]) == 2


if __name__ == "__main__":
    main()
