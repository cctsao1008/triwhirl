#!/usr/bin/env python3

from __future__ import annotations

import struct
import zlib

from tools.triwhirl_tool.standup_trace import (
    FRAME_END,
    FRAME_START,
    HEADER,
    RECORD,
    TRACE_MAGIC,
    parse_trace,
    trace_end_seen,
)


def frame(frame_seq: int, first_sample: int, records: list[bytes], flags: int = 0) -> bytes:
    payload = b"".join(records)
    crc = zlib.crc32(payload) & 0xFFFFFFFF if payload else 0
    return HEADER.pack(
        struct.unpack("<I", TRACE_MAGIC)[0],
        1,
        RECORD.size,
        len(records),
        flags,
        frame_seq,
        first_sample,
        0,
        0,
        crc,
    ) + payload


def record(seq: int, flags: int = 0x000B) -> bytes:
    return RECORD.pack(
        seq,
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
    blob = b"".join(
        (
            frame(0, 0, [], FRAME_START),
            frame(1, 0, [record(0), record(1)]),
            frame(2, 2, [record(2), record(3, 0x003B)]),
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
    assert len(parsed["records"]) == 4
    row = parsed["records"][0]
    assert row["sample_seq"] == 0
    assert abs(row["error_deg"] + 1.25) < 1e-9
    assert abs(row["theta_rate_rad_s"] - 1.5) < 1e-9
    assert abs(row["filtered_rate_rad_s"] - 1.0) < 1e-9
    assert abs(row["wheel_rate_rad_s"] + 23.0) < 1e-9
    assert row["phase"] == "balance"
    assert row["valid"]
    assert trace_end_seen(blob)

    missing = b"".join(
        (
            frame(0, 0, [], FRAME_START),
            frame(1, 0, [record(0), record(2)]),
            frame(2, 3, [], FRAME_END),
        )
    )
    missing_parsed = parse_trace(missing, tolerate_trailing=False)
    assert missing_parsed["sample_sequence_errors"] == 1
    assert missing_parsed["missing_samples"] == 1

    corrupted = bytearray(blob)
    data_start = HEADER.size + HEADER.size
    corrupted[data_start + 8] ^= 0x01
    corrupt_parsed = parse_trace(corrupted)
    assert corrupt_parsed["crc_errors"] >= 1


if __name__ == "__main__":
    main()
