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
    "phase",
    "theta_ref_rad",
    "planned_vq_v",
    "t_us",
    "theta_rad",
    "theta_rate_rad_s",
    "vel_rad_s",
    "vq_v",
]


def write_run(path: Path, theta_ref: float) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        for trial in (1, 2):
            trial_ref = theta_ref + 0.01 * (trial - 1)
            for sample in range(3):
                writer.writerow(
                    {
                        "schema_version": "2",
                        "trial": trial,
                        "phase": "active",
                        "theta_ref_rad": trial_ref,
                        "planned_vq_v": "0.25" if trial == 1 else "-0.25",
                        "t_us": trial * 10000 + sample * 1000,
                        "theta_rad": trial_ref,
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
        first = root / "first.csv"
        second = root / "second.csv"
        merged = root / "merged.csv"
        write_run(first, 0.40)
        write_run(second, -2.20)

        completed = subprocess.run(
            [sys.executable, str(SCRIPT), str(first), str(second), "-o", str(merged)],
            check=True,
            capture_output=True,
            text=True,
        )
        assert "active_merge" in completed.stdout

        with merged.open("r", newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        assert len(rows) == 12
        assert sorted({int(row["trial"]) for row in rows}) == [1, 2, 3, 4]
        assert "vertex_id" not in rows[0]
        assert {int(row["trial"]) for row in rows if float(row["theta_ref_rad"]) > 0.0} == {1, 2}
        assert {int(row["trial"]) for row in rows if float(row["theta_ref_rad"]) < 0.0} == {3, 4}

        manifest_path = merged.with_suffix(merged.suffix + ".merge.json")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        assert manifest["format"] == "triwhirl-active-merge-v2"
        assert manifest["rows"] == 12
        assert manifest["trials"] == 4
        assert manifest["legacy_vertices"] == []
        assert abs(manifest["theta_ref_min_rad"] - (-2.20)) < 1.0e-9
        assert abs(manifest["theta_ref_max_rad"] - 0.41) < 1.0e-9
        assert manifest["output_sha256"] == sha256(merged)
        assert len(manifest["sources"]) == 2
        assert manifest["sources"][0]["sha256"] == sha256(first)
        assert manifest["sources"][1]["sha256"] == sha256(second)
        assert manifest["sources"][0]["merged_trial_map"] == {"1": 1, "2": 2}
        assert manifest["sources"][1]["merged_trial_map"] == {"1": 3, "2": 4}

        bad = root / "bad.csv"
        bad.write_text("trial,theta_ref_rad,extra\n1,-1.0,x\n", encoding="utf-8")
        rejected = subprocess.run(
            [
                sys.executable,
                str(SCRIPT),
                str(first),
                str(bad),
                "-o",
                str(root / "bad-merged.csv"),
            ],
            capture_output=True,
            text=True,
        )
        assert rejected.returncode != 0
        assert "schema differs" in (rejected.stdout + rejected.stderr)

        missing_ref = root / "missing-ref.csv"
        missing_ref.write_text("trial,phase\n1,active\n", encoding="utf-8")
        rejected = subprocess.run(
            [sys.executable, str(SCRIPT), str(missing_ref), "-o", str(root / "x.csv")],
            capture_output=True,
            text=True,
        )
        assert rejected.returncode != 0
        assert "theta_ref_rad" in (rejected.stdout + rejected.stderr)

    print("active-ID merge: PASS")


if __name__ == "__main__":
    main()
