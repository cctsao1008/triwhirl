from __future__ import annotations

import csv
import math
import statistics
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

MAGIC = b"TWLG"
HEADER = struct.Struct("<IHHHHIIIII8I")
RECORD = struct.Struct("<IfffffIHH")
HEADER_BYTES = 64
RECORD_BYTES = 32

RECORD_ENCODER_VALID = 1 << 0
RECORD_WHEEL_RATE_VALID = 1 << 1
RECORD_IMU_VALID = 1 << 2
RECORD_ATTITUDE_VALID = 1 << 3

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
    theta_min_rad: float | None
    theta_max_rad: float | None
    max_abs_theta_rate_rad_s: float | None
    max_abs_wheel_rate_rad_s: float | None
    vq_min_v: float
    vq_max_v: float
    accel_weight_min: float | None
    accel_weight_max: float | None
    fault_or: int
    faulted_records: int
    flags_or: int
    encoder_valid_records: int
    wheel_rate_valid_records: int
    imu_valid_records: int
    attitude_valid_records: int
    actual_duration_s: float
    dt_min_us: int
    dt_max_us: int
    dt_mean_us: float
    dt_median_us: float
    dt_over_1250us: int
    dt_over_2000us: int


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
    theta_min: float | None = None
    theta_max: float | None = None
    max_abs_theta_rate: float | None = None
    max_abs_wheel_rate: float | None = None
    vq_min = math.inf
    vq_max = -math.inf
    accel_min: float | None = None
    accel_max: float | None = None
    fault_or = 0
    faulted_records = 0
    flags_or = 0
    encoder_valid_records = 0
    wheel_rate_valid_records = 0
    imu_valid_records = 0
    attitude_valid_records = 0
    previous_t_us: int | None = None
    deltas_us: list[int] = []
    count = 0

    for record in iter_records(payload):
        (
            t_us,
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

        if previous_t_us is not None:
            deltas_us.append((int(t_us) - previous_t_us) & 0xFFFFFFFF)
        previous_t_us = int(t_us)

        vq_min = min(vq_min, vq)
        vq_max = max(vq_max, vq)
        fault_or |= int(fault_mask)
        flags_or |= int(flags)
        if fault_mask:
            faulted_records += 1

        if flags & RECORD_ENCODER_VALID:
            encoder_valid_records += 1
        if flags & RECORD_WHEEL_RATE_VALID:
            wheel_rate_valid_records += 1
            max_abs_wheel_rate = max(
                0.0 if max_abs_wheel_rate is None else max_abs_wheel_rate,
                abs(wheel_rate),
            )
        if flags & RECORD_IMU_VALID:
            imu_valid_records += 1
        if flags & RECORD_ATTITUDE_VALID:
            attitude_valid_records += 1
            theta_min = theta if theta_min is None else min(theta_min, theta)
            theta_max = theta if theta_max is None else max(theta_max, theta)
            max_abs_theta_rate = max(
                0.0 if max_abs_theta_rate is None else max_abs_theta_rate,
                abs(theta_rate),
            )
            accel_min = (
                accel_weight if accel_min is None else min(accel_min, accel_weight)
            )
            accel_max = (
                accel_weight if accel_max is None else max(accel_max, accel_weight)
            )

    if count == 0:
        vq_min = vq_max = 0.0

    if deltas_us:
        actual_duration_s = sum(deltas_us) * 1.0e-6
        dt_min_us = min(deltas_us)
        dt_max_us = max(deltas_us)
        dt_mean_us = statistics.fmean(deltas_us)
        dt_median_us = float(statistics.median(deltas_us))
        dt_over_1250us = sum(delta > 1250 for delta in deltas_us)
        dt_over_2000us = sum(delta > 2000 for delta in deltas_us)
    else:
        actual_duration_s = 0.0
        dt_min_us = 0
        dt_max_us = 0
        dt_mean_us = 0.0
        dt_median_us = 0.0
        dt_over_1250us = 0
        dt_over_2000us = 0

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
        encoder_valid_records=encoder_valid_records,
        wheel_rate_valid_records=wheel_rate_valid_records,
        imu_valid_records=imu_valid_records,
        attitude_valid_records=attitude_valid_records,
        actual_duration_s=actual_duration_s,
        dt_min_us=dt_min_us,
        dt_max_us=dt_max_us,
        dt_mean_us=dt_mean_us,
        dt_median_us=dt_median_us,
        dt_over_1250us=dt_over_1250us,
        dt_over_2000us=dt_over_2000us,
    )
