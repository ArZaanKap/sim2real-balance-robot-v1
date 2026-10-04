#!/usr/bin/env python3
"""Summarise a static imu_noise capture using only the Python standard library."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Calculate static pitch and gyro noise statistics."
    )
    parser.add_argument("csv_file", type=Path, help="CSV written by capture.py")
    parser.add_argument(
        "--json",
        type=Path,
        default=None,
        help="stats JSON path (default: CSV name with .stats.json suffix)",
    )
    return parser.parse_args()


def percentile(sorted_values: list[float], fraction: float) -> float:
    if len(sorted_values) == 1:
        return sorted_values[0]
    position = fraction * (len(sorted_values) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return sorted_values[lower]
    weight = position - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


def describe(values: list[float]) -> dict[str, float]:
    ordered = sorted(values)
    mean = statistics.fmean(values)
    std = statistics.pstdev(values)
    return {
        "mean": mean,
        "std": std,
        "min": ordered[0],
        "p01": percentile(ordered, 0.01),
        "p05": percentile(ordered, 0.05),
        "median": percentile(ordered, 0.50),
        "p95": percentile(ordered, 0.95),
        "p99": percentile(ordered, 0.99),
        "max": ordered[-1],
        "peak_to_peak": ordered[-1] - ordered[0],
        # Uniform(-a, a) has std = a/sqrt(3).
        "equivalent_uniform_half_range": math.sqrt(3.0) * std,
    }


def print_signal(name: str, unit: str, stats: dict[str, float]) -> None:
    print(f"\n{name} [{unit}]")
    for key in ("mean", "std", "min", "p01", "p05", "median", "p95", "p99", "max", "peak_to_peak"):
        print(f"  {key:>14}: {stats[key]: .9g}")
    print(
        "  uniform +/- a: "
        f"{stats['equivalent_uniform_half_range']:.9g}  "
        "(same variance as measured std)"
    )


def main() -> int:
    args = parse_args()
    csv_path = args.csv_file.expanduser().resolve()
    json_path = (
        args.json.expanduser().resolve()
        if args.json is not None
        else csv_path.with_suffix(".stats.json")
    )

    ticks: list[int] = []
    times_us: list[int] = []
    sample_counts: list[int] = []
    pitch: list[float] = []
    gyro: list[float] = []
    accuracies: list[int] = []

    try:
        with csv_path.open(newline="", encoding="utf-8") as input_file:
            reader = csv.DictReader(input_file)
            required = {
                "tick",
                "time_us",
                "sample_count",
                "theta_y_rad",
                "gyro_y_rad_s",
                "accuracy",
            }
            if reader.fieldnames is None or not required.issubset(reader.fieldnames):
                raise ValueError(f"expected CSV columns: {', '.join(sorted(required))}")

            for row_number, row in enumerate(reader, start=2):
                try:
                    ticks.append(int(row["tick"]))
                    times_us.append(int(row["time_us"]))
                    sample_counts.append(int(row["sample_count"]))
                    pitch.append(float(row["theta_y_rad"]))
                    gyro.append(float(row["gyro_y_rad_s"]))
                    accuracies.append(int(row["accuracy"]))
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"invalid value on CSV row {row_number}: {exc}") from exc
    except (OSError, ValueError) as exc:
        print(f"Could not analyse {csv_path}: {exc}")
        return 2

    if len(pitch) < 2:
        print("Need at least two samples.")
        return 1

    pitch_stats = describe(pitch)
    gyro_stats = describe(gyro)

    time_deltas = [b - a for a, b in zip(times_us, times_us[1:]) if b > a]
    duration_s = (times_us[-1] - times_us[0]) / 1_000_000.0
    measured_hz = (len(times_us) - 1) / duration_s if duration_s > 0 else float("nan")
    duplicate_quaternions = sum(
        current == previous
        for previous, current in zip(sample_counts, sample_counts[1:])
    )
    duplicate_fraction = duplicate_quaternions / (len(sample_counts) - 1)

    result = {
        "source": str(csv_path),
        "samples": len(pitch),
        "duration_s": duration_s,
        "measured_control_rate_hz": measured_hz,
        "mean_period_us": statistics.fmean(time_deltas) if time_deltas else None,
        "duplicate_quaternion_fraction": duplicate_fraction,
        "accuracy_counts": {str(value): accuracies.count(value) for value in sorted(set(accuracies))},
        "theta_y_rad": pitch_stats,
        "gyro_y_rad_s": gyro_stats,
    }

    print(f"File:       {csv_path}")
    print(f"Samples:    {len(pitch)}")
    print(f"Duration:   {duration_s:.3f} s")
    print(f"Rate:       {measured_hz:.3f} Hz")
    print(f"Repeated quaternion snapshots: {duplicate_fraction:.2%}")
    print(f"Accuracy counts (0=low, 3=high): {result['accuracy_counts']}")
    print_signal("theta_y", "rad", pitch_stats)
    print_signal("gyro_y", "rad/s", gyro_stats)

    print("\nInterpretation")
    print("  theta_y mean is a mounting/fixture offset only if the robot was truly upright.")
    print("  gyro_y mean is stationary gyro bias, not zero-mean sample noise.")
    print("  Use the reported uniform +/- a values to replace the current guessed")
    print("  per-sample uniform ranges while preserving the measured variance.")

    try:
        json_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    except OSError as exc:
        print(f"Could not write JSON result {json_path}: {exc}")
        return 2
    print(f"\nSaved machine-readable statistics to {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
