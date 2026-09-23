from __future__ import annotations

import csv
import json
import struct
import subprocess
import time
import zlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

TRACE_MAGIC = b"TWTR"
TRACE_VERSION = 1
HEADER = struct.Struct("<IBBBBIIIII")
RECORD = struct.Struct("<IhhhhhhhhhH")
FRAME_START = 1 << 0
FRAME_END = 1 << 1
FRAME_OVERRUN = 1 << 2
PHASE_NAMES = {
    0: "idle",
    1: "swing_high",
    2: "swing_low",
    3: "balance",
}


@dataclass
class StandupTraceCapture:
    """BLE callback sink: receive bytes into host RAM and do nothing expensive."""

    raw: bytearray = field(default_factory=bytearray)
    notification_count: int = 0
    first_notify_ns: int | None = None
    last_notify_ns: int | None = None
    max_notify_gap_ns: int = 0

    def on_notify(self, _sender: Any, data: bytearray) -> None:
        now_ns = time.monotonic_ns()
        if self.first_notify_ns is None:
            self.first_notify_ns = now_ns
        if self.last_notify_ns is not None:
            gap = now_ns - self.last_notify_ns
            if gap > self.max_notify_gap_ns:
                self.max_notify_gap_ns = gap
        self.last_notify_ns = now_ns
        self.notification_count += 1
        self.raw.extend(data)


def _git_head() -> str | None:
    try:
        completed = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError:
        return None
    value = completed.stdout.strip()
    return value if completed.returncode == 0 and value else None


def parse_trace(raw: bytes | bytearray, *, tolerate_trailing: bool = True) -> dict[str, Any]:
    data = bytes(raw)
    offset = 0
    frames: list[dict[str, Any]] = []
    records: list[dict[str, Any]] = []
    framing_skipped = 0
    crc_errors = 0
    frame_sequence_errors = 0
    sample_sequence_errors = 0
    missing_samples = 0
    duplicate_or_reordered_samples = 0
    expected_frame_seq: int | None = None
    expected_sample_seq: int | None = None
    start_seen = False
    end_seen = False
    max_dropped_records = 0
    max_transport_dropped_bytes = 0

    while offset + HEADER.size <= len(data):
        if data[offset : offset + 4] != TRACE_MAGIC:
            next_magic = data.find(TRACE_MAGIC, offset + 1)
            if next_magic < 0:
                break
            framing_skipped += next_magic - offset
            offset = next_magic
            continue

        (
            magic,
            version,
            record_size,
            sample_count,
            frame_flags,
            frame_seq,
            first_sample_seq,
            dropped_records,
            transport_dropped_bytes,
            payload_crc32,
        ) = HEADER.unpack_from(data, offset)
        if magic != int.from_bytes(TRACE_MAGIC, "little"):
            offset += 1
            framing_skipped += 1
            continue
        if version != TRACE_VERSION or record_size != RECORD.size or sample_count > 8:
            offset += 1
            framing_skipped += 1
            continue

        frame_bytes = HEADER.size + sample_count * record_size
        if offset + frame_bytes > len(data):
            if tolerate_trailing:
                break
            raise RuntimeError("truncated standup trace frame")

        payload = data[offset + HEADER.size : offset + frame_bytes]
        crc_ok = sample_count == 0 or (zlib.crc32(payload) & 0xFFFFFFFF) == payload_crc32
        if not crc_ok:
            crc_errors += 1

        if expected_frame_seq is not None and frame_seq != expected_frame_seq:
            frame_sequence_errors += 1
        expected_frame_seq = (frame_seq + 1) & 0xFFFFFFFF

        start_seen = start_seen or bool(frame_flags & FRAME_START)
        end_seen = end_seen or bool(frame_flags & FRAME_END)
        max_dropped_records = max(max_dropped_records, dropped_records)
        max_transport_dropped_bytes = max(
            max_transport_dropped_bytes, transport_dropped_bytes
        )

        frame_record_start = len(records)
        for index in range(sample_count):
            values = RECORD.unpack_from(payload, index * record_size)
            (
                sample_seq,
                error_cdeg,
                theta_rate_mrad_s,
                filtered_rate_mrad_s,
                wheel_rate_centi_rad_s,
                target_velocity_centi_rad_s,
                velocity_error_centi_rad_s,
                integral_mv,
                vq_target_mv,
                vq_applied_mv,
                flags,
            ) = values

            if expected_sample_seq is not None and sample_seq != expected_sample_seq:
                sample_sequence_errors += 1
                delta = (sample_seq - expected_sample_seq) & 0xFFFFFFFF
                if 0 < delta < 0x80000000:
                    missing_samples += delta
                else:
                    duplicate_or_reordered_samples += 1
            expected_sample_seq = (sample_seq + 1) & 0xFFFFFFFF

            phase_id = flags & 0x0003
            records.append(
                {
                    "sample_seq": sample_seq,
                    "t_s": sample_seq * 0.001,
                    "error_deg": error_cdeg * 0.01,
                    "theta_rate_rad_s": theta_rate_mrad_s * 0.001,
                    "filtered_rate_rad_s": filtered_rate_mrad_s * 0.001,
                    "wheel_rate_rad_s": wheel_rate_centi_rad_s * 0.01,
                    "target_velocity_rad_s": target_velocity_centi_rad_s * 0.01,
                    "velocity_error_rad_s": velocity_error_centi_rad_s * 0.01,
                    "velocity_integral_v": integral_mv * 0.001,
                    "vq_target_v": vq_target_mv * 0.001,
                    "vq_applied_v": vq_applied_mv * 0.001,
                    "phase": PHASE_NAMES.get(phase_id, f"unknown_{phase_id}"),
                    "stable": bool(flags & (1 << 2)),
                    "valid": bool(flags & (1 << 3)),
                    "target_saturated": bool(flags & (1 << 4)),
                    "vq_saturated": bool(flags & (1 << 5)),
                    "safety_fault": bool(flags & (1 << 6)),
                    "flags": flags,
                }
            )

        frames.append(
            {
                "frame_seq": frame_seq,
                "first_sample_seq": first_sample_seq,
                "sample_count": sample_count,
                "flags": frame_flags,
                "crc_ok": crc_ok,
                "dropped_records": dropped_records,
                "transport_dropped_bytes": transport_dropped_bytes,
                "record_start": frame_record_start,
            }
        )
        offset += frame_bytes

    return {
        "frames": frames,
        "records": records,
        "bytes_consumed": offset,
        "bytes_total": len(data),
        "trailing_bytes": len(data) - offset,
        "framing_skipped_bytes": framing_skipped,
        "crc_errors": crc_errors,
        "frame_sequence_errors": frame_sequence_errors,
        "sample_sequence_errors": sample_sequence_errors,
        "missing_samples": missing_samples,
        "duplicate_or_reordered_samples": duplicate_or_reordered_samples,
        "start_seen": start_seen,
        "end_seen": end_seen,
        "max_dropped_records": max_dropped_records,
        "max_transport_dropped_bytes": max_transport_dropped_bytes,
    }


def trace_end_seen(raw: bytes | bytearray) -> bool:
    # End marker is a header-only frame. Scanning from the last magic keeps this
    # cheap while the host waits for the Core-0 transport to flush after stop.
    data = bytes(raw)
    pos = data.rfind(TRACE_MAGIC)
    while pos >= 0:
        if pos + HEADER.size <= len(data):
            header = HEADER.unpack_from(data, pos)
            version = header[1]
            record_size = header[2]
            sample_count = header[3]
            flags = header[4]
            frame_bytes = HEADER.size + sample_count * record_size
            if (
                version == TRACE_VERSION
                and record_size == RECORD.size
                and sample_count <= 8
                and pos + frame_bytes <= len(data)
                and flags & FRAME_END
            ):
                return True
        pos = data.rfind(TRACE_MAGIC, 0, pos)
    return False


def save_trace(capture: StandupTraceCapture, prefix: Path) -> tuple[Path, Path, Path, dict[str, Any]]:
    prefix.parent.mkdir(parents=True, exist_ok=True)
    parsed = parse_trace(capture.raw)
    raw_path = prefix.with_suffix(".twtrace")
    csv_path = prefix.with_suffix(".csv")
    json_path = prefix.with_suffix(".json")

    raw_path.write_bytes(capture.raw)

    records = parsed["records"]
    fieldnames = list(records[0].keys()) if records else [
        "sample_seq",
        "t_s",
        "error_deg",
        "theta_rate_rad_s",
        "filtered_rate_rad_s",
        "wheel_rate_rad_s",
        "target_velocity_rad_s",
        "velocity_error_rad_s",
        "velocity_integral_v",
        "vq_target_v",
        "vq_applied_v",
        "phase",
        "stable",
        "valid",
        "target_saturated",
        "vq_saturated",
        "safety_fault",
        "flags",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)

    duration_s = 0.0
    if capture.first_notify_ns is not None and capture.last_notify_ns is not None:
        duration_s = max(0.0, (capture.last_notify_ns - capture.first_notify_ns) * 1e-9)
    summary = {
        "format": "TWTR1",
        "firmware_git_head": _git_head(),
        "raw_bytes": len(capture.raw),
        "notifications": capture.notification_count,
        "receive_duration_s": duration_s,
        "receive_throughput_kB_s": (
            len(capture.raw) / duration_s / 1000.0 if duration_s > 0.0 else 0.0
        ),
        "max_notification_gap_ms": capture.max_notify_gap_ns * 1e-6,
        "frames": len(parsed["frames"]),
        "records": len(records),
        "start_seen": parsed["start_seen"],
        "end_seen": parsed["end_seen"],
        "crc_errors": parsed["crc_errors"],
        "frame_sequence_errors": parsed["frame_sequence_errors"],
        "sample_sequence_errors": parsed["sample_sequence_errors"],
        "missing_samples": parsed["missing_samples"],
        "duplicate_or_reordered_samples": parsed["duplicate_or_reordered_samples"],
        "ring_dropped_records": parsed["max_dropped_records"],
        "transport_dropped_bytes": parsed["max_transport_dropped_bytes"],
        "trailing_bytes": parsed["trailing_bytes"],
        "framing_skipped_bytes": parsed["framing_skipped_bytes"],
    }
    json_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return raw_path, csv_path, json_path, summary
