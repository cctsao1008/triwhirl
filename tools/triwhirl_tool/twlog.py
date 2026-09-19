from __future__ import annotations

import csv
import math
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator

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


@dataclass(frozen=True)
class TwLogMeta:
    version: int
    header_size: int
    record_size: int
    sample_period_us: int
    record_count: int
    payload_bytes: int
    dropped_records: int
    payload_crc32: int
    flags: int
    physical_payload_offset: int
    prepared_bytes: int

    @property
    def duration_s(self) -> float:
        if self.record_count <= 1:
            return 0.0
        return (self.record_count - 1) * self.sample_period_us * 1.0e-6

    @property
    def sample_rate_hz(self) -> float:
        if self.sample_period_us <= 0:
            return 0.0
        return 1.0e6 / self.sample_period_us


@dataclass(frozen=True)
class TwLogStats:
    theta_min_rad: float
    theta_max_rad: float
    max_abs_theta_rate_rad_s: float
    max_abs_wheel_rate_rad_s: float
    vq_min_v: float
    vq_max_v: float
    accel_weight_min: float
    accel_weight_max: float
    fault_or: int
    faulted_records: int
    flags_or: int


def decode_bytes(blob: bytes, *, source: str = "TWLG") -> tuple[TwLogMeta, bytes]:
    if len(blob) < HEADER_BYTES:
        raise RuntimeError(f"{source}: file is shorter than the 64-byte TWLG header")
    if blob[:4] != MAGIC:
        raise RuntimeError(f"{source}: bad TWLG magic {blob[:4]!r}")

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

    meta = TwLogMeta(
        version=version,
        header_size=header_size,
        record_size=record_size,
        sample_period_us=sample_period_us,
        record_count=record_count,
        payload_bytes=payload_bytes,
        dropped_records=dropped_records,
        payload_crc32=payload_crc32,
        flags=flags,
        physical_payload_offset=reserved[0],
        prepared_bytes=reserved[1],
    )
    return meta, payload


def read(path: Path) -> tuple[TwLogMeta, bytes]:
    return decode_bytes(path.read_bytes(), source=str(path))


def iter_records(payload: bytes) -> Iterator[tuple[int, float, float, float, float, float, int, int, int]]:
    if len(payload) % RECORD_BYTES != 0:
        raise RuntimeError("TWLG payload is not record aligned")
    for offset in range(0, len(payload), RECORD_BYTES):
        yield RECORD.unpack_from(payload, offset)


def write_csv(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(FIELDS)
        writer.writerows(iter_records(payload))


def summarize(payload: bytes) -> TwLogStats:
    theta_min = math.inf
    theta_max = -math.inf
    max_abs_theta_rate = 0.0
    max_abs_wheel_rate = 0.0
    vq_min = math.inf
    vq_max = -math.inf
    accel_min = math.inf
    accel_max = -math.inf
    fault_or = 0
    faulted_records = 0
    flags_or = 0
    count = 0

    for record in iter_records(payload):
        (
            _t_us,
            theta,
            theta_rate,
            wheel_rate,
            vq,
            accel_weight,
            fault_mask,
            flags,
            _raw_count,
        ) = record
        count += 1
        theta_min = min(theta_min, theta)
        theta_max = max(theta_max, theta)
        max_abs_theta_rate = max(max_abs_theta_rate, abs(theta_rate))
        max_abs_wheel_rate = max(max_abs_wheel_rate, abs(wheel_rate))
        vq_min = min(vq_min, vq)
        vq_max = max(vq_max, vq)
        accel_min = min(accel_min, accel_weight)
        accel_max = max(accel_max, accel_weight)
        fault_or |= int(fault_mask)
        flags_or |= int(flags)
        if fault_mask:
            faulted_records += 1

    if count == 0:
        theta_min = theta_max = 0.0
        vq_min = vq_max = 0.0
        accel_min = accel_max = 0.0

    return TwLogStats(
        theta_min_rad=theta_min,
        theta_max_rad=theta_max,
        max_abs_theta_rate_rad_s=max_abs_theta_rate,
        max_abs_wheel_rate_rad_s=max_abs_wheel_rate,
        vq_min_v=vq_min,
        vq_max_v=vq_max,
        accel_weight_min=accel_min,
        accel_weight_max=accel_max,
        fault_or=fault_or,
        faulted_records=faulted_records,
        flags_or=flags_or,
    )
