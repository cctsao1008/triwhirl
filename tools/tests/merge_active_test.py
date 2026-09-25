#!/usr/bin/env python3

from __future__ import annotations

import csv
import hashlib
import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools/parameter_id/merge_active.py"

FIELDS = [
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


def write_run(path: Path, vertex: str, theta_ref: float) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for trial in (1, 2):
            for sample in range(3):
                writer.writerow(
                    {
                        "schema_version": "1",
                        "trial": trial,
                        "vertex_id": vertex,
                        "vertex_center_deg": {"A": "68.0", "B": "-52.0"}[vertex],
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


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    with tempfile.TemporaryDirectory() as temporary:
        root = Path(temporary)
        a = root / "a.csv"
        b = root / "b.csv"
        merged = root / "merged.csv"
        write_run(a, "A", 1.18)
        write_run(b, "B", -0.91)

        completed = subprocess.run(
            [sys.executable, str(SCRIPT), str(a), str(b), "-o", str(merged)],
            check=True,
            capture_output=True,
            text=True,
        )
        assert "active_merge" in completed.stdout

        with merged.open("r", newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 12
        assert sorted({int(row["trial"]) for row in rows}) == [1, 2, 3, 4]
        assert {int(row["trial"]) for row in rows if row["vertex_id"] == "A"} == {1, 2}
        assert {int(row["trial"]) for row in rows if row["vertex_id"] == "B"} == {3, 4}

        manifest_path = merged.with_suffix(merged.suffix + ".merge.json")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        assert manifest["format"] == "triwhirl-active-merge-v1"
        assert manifest["rows"] == 12
        assert manifest["trials"] == 4
        assert manifest["vertices"] == ["A", "B"]
        assert manifest["output_sha256"] == sha256(merged)
        assert len(manifest["sources"]) == 2
        assert manifest["sources"][0]["sha256"] == sha256(a)
        assert manifest["sources"][1]["sha256"] == sha256(b)
        assert manifest["sources"][0]["merged_trial_map"] == {"1": 1, "2": 2}
        assert manifest["sources"][1]["merged_trial_map"] == {"1": 3, "2": 4}

        bad = root / "bad.csv"
        bad.write_text("trial,vertex_id,extra\n1,B,x\n", encoding="utf-8")
        rejected = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                str(a),
                str(bad),
                "-o",
                str(root / "bad-merged.csv"),
            ],
            capture_output=True,
            text=True,
        )
        assert rejected.returncode != 0
        assert "schema differs" in (rejected.stdout + rejected.stderr)

    print("active-ID merge: PASS")


if __name__ == "__main__":
    main()
