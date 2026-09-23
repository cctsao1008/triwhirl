#!/usr/bin/env python3

from __future__ import annotations

import json
import struct
import sys
import tempfile
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from tools.triwhirl_tool.standup_trace import (  # noqa: E402
    FRAME_END,
    FRAME_FIRMWARE_DIRTY,
    FRAME_START,
    HEADER,
    RECORD,
    RECORD_DT_CLAMPED,
    TRACE_MAGIC,
    TRACE_RECORDS_PER_FRAME,
    TRACE_VERSION,
    StandupTraceCapture,
    parse_trace,
    save_trace,
    trace_end_seen,
)

FIRMWARE_WORDS = (0x12345678, 0x90ABCDEF, 0x00112233, 0x44556677, 0x8899AABB)
FIRMWARE_HEAD = "1234567890abcdef00112233445566778899aabb"


def frame(
    frame_seq: int,
    first_sample: int,
    records: list[bytes],
    flags: int = 0,
    firmware_words: tuple[int, int, int, int, int] = FIRMWARE_WORDS,
) -> bytes:
    payload = b"".join(records)
    crc = zlib.crc32(payload) & 0xFFFFFFFF if payload else 0
    return HEADER.pack(
        struct.unpack("<I", TRACE_MAGIC)[0],
        TRACE_VERSION,
        RECORD.size,
        len(records),
        flags,
        frame_seq,
        first_sample,
        0,
        0,
        *firmware_words,
        crc,
    ) + payload


def record(seq: int, dt_us: int, flags: int = 0x000B) -> bytes:
    return RECORD.pack(
        seq,
        dt_us,
        -125,
        1500,
        1000,
        -2300,
        -4000,
        -1700,
        -250,
        -1200,
        -1000,
        flags,
    )


def main() -> None:
    assert HEADER.size == 48
    assert RECORD.size == 26
    assert TRACE_RECORDS_PER_FRAME == 7
    assert HEADER.size + TRACE_RECORDS_PER_FRAME * RECORD.size == 230

    blob = b"".join(
        (
            frame(0, 0, [], FRAME_START),
            frame(1, 0, [record(0, 0), record(1, 995)]),
            frame(2, 2, [record(2, 1005), record(3, 1000, 0x003B)]),
            frame(3, 4, [], FRAME_END),
        )
    )
    parsed = parse_trace(blob, tolerate_trailing=False)
    assert parsed["start_seen"]
    assert parsed["end_seen"]
    assert parsed["crc_errors"] == 0
    assert parsed["frame_sequence_errors"] == 0
    assert parsed["sample_sequence_errors"] == 0
    assert parsed["missing_samples"] == 0
    assert parsed["firmware_git_head"] == FIRMWARE_HEAD
    assert not parsed["firmware_dirty"]
    assert parsed["firmware_identity_errors"] == 0
    assert parsed["dt_min_us"] == 995
    assert parsed["dt_max_us"] == 1005
    assert abs(parsed["dt_mean_us"] - 1000.0) < 1e-9
    assert abs(parsed["control_duration_s"] - 0.003) < 1e-12
    assert len(parsed["records"]) == 4
    row = parsed["records"][0]
    assert row["sample_seq"] == 0
    assert row["dt_us"] == 0
    assert row["t_s"] == 0.0
    assert abs(row["error_deg"] + 1.25) < 1e-9
    assert abs(row["theta_rate_rad_s"] - 1.5) < 1e-9
    assert abs(row["filtered_rate_rad_s"] - 1.0) < 1e-9
    assert abs(row["wheel_rate_rad_s"] + 23.0) < 1e-9
    assert row["phase"] == "balance"
    assert row["valid"]
    assert abs(parsed["records"][1]["t_s"] - 0.000995) < 1e-12
    assert trace_end_seen(blob)

    missing = b"".join(
        (
            frame(0, 0, [], FRAME_START),
            frame(1, 0, [record(0, 0), record(2, 1000)]),
            frame(2, 3, [], FRAME_END),
        )
    )
    missing_parsed = parse_trace(missing, tolerate_trailing=False)
    assert missing_parsed["sample_sequence_errors"] == 1
    assert missing_parsed["missing_samples"] == 1

    dirty_clamped = b"".join(
        (
            frame(0, 0, [], FRAME_START | FRAME_FIRMWARE_DIRTY),
            frame(
                1,
                0,
                [record(0, 0), record(1, 0xFFFF, 0x000B | RECORD_DT_CLAMPED)],
                FRAME_FIRMWARE_DIRTY,
            ),
            frame(2, 2, [], FRAME_END | FRAME_FIRMWARE_DIRTY),
        )
    )
    dirty_parsed = parse_trace(dirty_clamped, tolerate_trailing=False)
    assert dirty_parsed["firmware_dirty"]
    assert dirty_parsed["dt_clamped_records"] == 1

    inconsistent_words = (
        0xDEADBEEF,
        FIRMWARE_WORDS[1],
        FIRMWARE_WORDS[2],
        FIRMWARE_WORDS[3],
        FIRMWARE_WORDS[4],
    )
    inconsistent = b"".join(
        (
            frame(0, 0, [], FRAME_START),
            frame(1, 0, [record(0, 0)], firmware_words=inconsistent_words),
            frame(2, 1, [], FRAME_END),
        )
    )
    assert parse_trace(inconsistent)["firmware_identity_errors"] == 1

    corrupted = bytearray(blob)
    data_start = HEADER.size + HEADER.size
    corrupted[data_start + 10] ^= 0x01
    corrupt_parsed = parse_trace(corrupted)
    assert corrupt_parsed["crc_errors"] >= 1

    capture = StandupTraceCapture()
    capture.raw.extend(blob)
    with tempfile.TemporaryDirectory() as temporary:
        prefix = Path(temporary) / "trace"
        _raw, _csv, json_path, summary = save_trace(
            capture,
            prefix,
            run_metadata={"requested_duration_s": 1.0},
        )
        assert summary["format"] == "TWTR2"
        assert summary["firmware_git_head"] == FIRMWARE_HEAD
        assert summary["provenance_ok"]
        assert summary["trace_lossless"]
        assert summary["control_dt_min_us"] == 995
        assert summary["control_dt_max_us"] == 1005
        saved = json.loads(json_path.read_text(encoding="utf-8"))
        assert saved["run"]["requested_duration_s"] == 1.0


if __name__ == "__main__":
    main()
