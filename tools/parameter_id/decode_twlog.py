#!/usr/bin/env python3
"""Decode a TriWhirl TWLG v1 binary runtime log to CSV."""

from __future__ import annotations

import argparse
import csv
import struct
import zlib
from pathlib import Path

MAGIC = b"TWLG"
HEADER = struct.Struct("<IHHHHIIIII8I")
RECORD = struct.Struct("<IfffffIHH")
HEADER_BYTES = 64
RECORD_BYTES = 32

FIELDS = (
    "t_us",
    "theta_rad",
    "theta_rate_rad_s",
    "wheel_rate_rad_s",
    "vq_v",
    "accel_weight",
    "fault_mask",
    "flags",
    "raw_count",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Decode TWLG v1 to CSV")
    parser.add_argument("input", type=Path)
    parser.add_argument("-o", "--output", type=Path, required=True)
    return parser.parse_args()


def decode(path: Path) -> tuple[dict[str, int], bytes]:
    blob = path.read_bytes()
    if len(blob) < HEADER_BYTES:
        raise RuntimeError(f"{path}: file is shorter than the 64-byte TWLG header")
    if blob[:4] != MAGIC:
        raise RuntimeError(f"{path}: bad TWLG magic {blob[:4]!r}")

    values = HEADER.unpack_from(blob, 0)
    (
        magic_u32,
        version,
        header_size,
        record_size,
        sample_period_us,
        record_count,
        payload_bytes,
        dropped_records,
        payload_crc32,
        flags,
        *reserved,
    ) = values
    del magic_u32

    if version != 1:
        raise RuntimeError(f"unsupported TWLG version {version}")
    if header_size != HEADER_BYTES:
        raise RuntimeError(f"unexpected header size {header_size}")
    if record_size != RECORD_BYTES:
        raise RuntimeError(f"unexpected record size {record_size}")
    if payload_bytes != record_count * RECORD_BYTES:
        raise RuntimeError(
            f"payload size mismatch: header={payload_bytes}, records={record_count}"
        )
    expected_size = header_size + payload_bytes
    if len(blob) != expected_size:
        raise RuntimeError(
            f"file size mismatch: got {len(blob)} bytes, expected {expected_size}"
        )

    payload = blob[header_size:]
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    if crc != payload_crc32:
        raise RuntimeError(
            f"payload CRC mismatch: got 0x{crc:08x}, expected 0x{payload_crc32:08x}"
        )

    meta = {
        "version": version,
        "header_size": header_size,
        "record_size": record_size,
        "sample_period_us": sample_period_us,
        "record_count": record_count,
        "payload_bytes": payload_bytes,
        "dropped_records": dropped_records,
        "payload_crc32": payload_crc32,
        "flags": flags,
        "physical_payload_offset": reserved[0],
        "prepared_bytes": reserved[1],
    }
    return meta, payload


def main() -> int:
    args = parse_args()
    try:
        meta, payload = decode(args.input)
    except (OSError, RuntimeError) as exc:
        print(f"error: {exc}")
        return 1

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(FIELDS)
        for offset in range(0, len(payload), RECORD_BYTES):
            writer.writerow(RECORD.unpack_from(payload, offset))

    print(
        f"TWLG v{meta['version']}: {meta['record_count']} records, "
        f"Ts={meta['sample_period_us']} us, dropped={meta['dropped_records']}, "
        f"CRC=0x{meta['payload_crc32']:08x}"
    )
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
