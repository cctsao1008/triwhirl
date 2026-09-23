from __future__ import annotations

import csv
from pathlib import Path
from typing import Any, Iterable

from .standup_trace import parse_trace


def load_standup_records(path: Path) -> list[dict[str, Any]]:
    """Load decoded standup records from a .twtrace or generated .csv file."""
    suffix = path.suffix.lower()
    if suffix == ".twtrace":
        parsed = parse_trace(path.read_bytes(), tolerate_trailing=False)
        if not parsed["records"]:
            raise RuntimeError(f"no standup records in {path}")
        return list(parsed["records"])
    if suffix == ".csv":
        with path.open(newline="", encoding="utf-8-sig") as stream:
            rows = list(csv.DictReader(stream))
        if not rows:
            raise RuntimeError(f"no standup records in {path}")
        numeric = {
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
            "flags",
        }
        boolean = {
            "stable",
            "valid",
            "target_saturated",
            "vq_saturated",
            "safety_fault",
        }
        result: list[dict[str, Any]] = []
        for row in rows:
            item: dict[str, Any] = dict(row)
            for key in numeric:
                if key not in item or item[key] in (None, ""):
                    continue
                value = float(item[key])
                item[key] = int(value) if key in {"sample_seq", "flags"} else value
            for key in boolean:
                if key in item:
                    item[key] = str(item[key]).strip().lower() in {"1", "true", "yes"}
            result.append(item)
        return result
    raise RuntimeError("standup plot input must be .twtrace or .csv")


def _balance_spans(records: list[dict[str, Any]]) -> Iterable[tuple[float, float]]:
    start: float | None = None
    previous_t: float | None = None
    for row in records:
        t = float(row["t_s"])
        balance = row.get("phase") == "balance"
        if balance and start is None:
            start = t
        elif not balance and start is not None:
            yield start, previous_t if previous_t is not None else t
            start = None
        previous_t = t
    if start is not None and previous_t is not None:
        yield start, previous_t


def plot_standup_records(
    records: list[dict[str, Any]],
    output: Path,
    *,
    show: bool = False,
    title: str | None = None,
) -> Path:
    """Render the closed-loop standup trace after acquisition has finished."""
    if not records:
        raise RuntimeError("cannot plot an empty standup trace")
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "matplotlib is required for plotting; install it with: python -m pip install matplotlib"
        ) from exc

    t = [float(row["t_s"]) for row in records]
    error = [float(row["error_deg"]) for row in records]
    theta_rate = [float(row["theta_rate_rad_s"]) for row in records]
    filtered_rate = [float(row["filtered_rate_rad_s"]) for row in records]
    wheel_rate = [float(row["wheel_rate_rad_s"]) for row in records]
    target_velocity = [float(row["target_velocity_rad_s"]) for row in records]
    vq_target = [float(row["vq_target_v"]) for row in records]
    vq_applied = [float(row["vq_applied_v"]) for row in records]
    integral = [float(row["velocity_integral_v"]) for row in records]

    figure, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True, constrained_layout=True)
    figure.suptitle(title or "TriWhirl standup closed-loop trace")

    axes[0].plot(t, error, label="upright error")
    axes[0].axhline(9.0, linestyle="--", linewidth=0.8, label="capture ±9°")
    axes[0].axhline(-9.0, linestyle="--", linewidth=0.8)
    axes[0].axhline(12.0, linestyle=":", linewidth=0.8, label="release ±12°")
    axes[0].axhline(-12.0, linestyle=":", linewidth=0.8)
    axes[0].set_ylabel("Error [deg]")
    axes[0].legend(loc="upper right")

    axes[1].plot(t, theta_rate, label="body rate")
    axes[1].plot(t, filtered_rate, label="vendor filtered rate")
    axes[1].set_ylabel("Rate [rad/s]")
    axes[1].legend(loc="upper right")

    axes[2].plot(t, wheel_rate, label="wheel rate")
    axes[2].plot(t, target_velocity, label="LQR target velocity")
    axes[2].set_ylabel("Wheel [rad/s]")
    axes[2].legend(loc="upper right")

    axes[3].plot(t, vq_target, label="Vq target")
    axes[3].plot(t, vq_applied, label="Vq applied")
    axes[3].plot(t, integral, label="velocity PI integral")
    axes[3].set_ylabel("Voltage [V]")
    axes[3].set_xlabel("Time [s]")
    axes[3].legend(loc="upper right")

    spans = list(_balance_spans(records))
    for axis in axes:
        axis.grid(True, alpha=0.25)
        for begin, end in spans:
            axis.axvspan(begin, end, alpha=0.08)

    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    if show:
        plt.show()
    plt.close(figure)
    return output


def plot_standup_trace(
    input_path: Path,
    output: Path | None = None,
    *,
    show: bool = False,
    title: str | None = None,
) -> Path:
    records = load_standup_records(input_path)
    destination = output if output is not None else input_path.with_suffix(".png")
    return plot_standup_records(records, destination, show=show, title=title)
