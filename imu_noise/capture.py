#!/usr/bin/env python3
"""Capture 100 Hz IMU CSV rows from the ESP32 without third-party packages."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import select
import subprocess
import sys
import time


FIELDS = (
    "tick",
    "time_us",
    "sample_count",
    "theta_y_rad",
    "gyro_y_rad_s",
    "accuracy",
)


def default_output() -> Path:
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parent / "logs" / f"imu_static_{stamp}.csv"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture IMU, rows from the imu_noise ESP32 firmware."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="serial device")
    parser.add_argument(
        "--duration",
        type=float,
        default=120.0,
        help="recording seconds after warm-up; <=0 records until Ctrl-C (default: 120)",
    )
    parser.add_argument(
        "--warmup",
        type=float,
        default=10.0,
        help="seconds of valid data to discard first (default: 10)",
    )
    parser.add_argument("--output", type=Path, default=None, help="output CSV path")
    return parser.parse_args()


def valid_values(parts: list[str]) -> bool:
    if len(parts) != len(FIELDS):
        return False
    try:
        int(parts[0])
        int(parts[1])
        int(parts[2])
        float(parts[3])
        float(parts[4])
        int(parts[5])
    except ValueError:
        return False
    return True


def main() -> int:
    args = parse_args()
    if args.warmup < 0:
        raise SystemExit("--warmup cannot be negative")

    output = (args.output or default_output()).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    try:
        subprocess.run(
            ["stty", "-F", args.port, "115200", "raw", "-echo", "-crtscts"],
            check=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        print(f"Could not configure {args.port}: {exc}", file=sys.stderr)
        return 2

    print(f"Waiting for IMU data on {args.port} ...")
    print("Keep the robot rigidly fixed at its known upright angle.")

    valid_start: float | None = None
    saved = 0
    malformed = 0
    last_report = time.monotonic()

    try:
        with open(args.port, "rb", buffering=0) as serial_port, output.open(
            "w", newline="", encoding="utf-8"
        ) as output_file:
            writer = csv.writer(output_file)
            writer.writerow(FIELDS)

            while True:
                ready, _, _ = select.select([serial_port], [], [], 1.0)
                if not ready:
                    continue

                raw = serial_port.readline()
                if not raw:
                    continue

                line = raw.decode("ascii", errors="ignore").strip()
                marker = line.find("IMU,")
                if marker < 0:
                    continue

                parts = line[marker + 4 :].split(",")
                if not valid_values(parts):
                    malformed += 1
                    continue

                now = time.monotonic()
                if valid_start is None:
                    valid_start = now

                elapsed = now - valid_start
                if elapsed < args.warmup:
                    if now - last_report >= 1.0:
                        print(f"Warm-up: {elapsed:5.1f}/{args.warmup:.1f} s", end="\r")
                        last_report = now
                    continue

                if args.duration > 0 and elapsed >= args.warmup + args.duration:
                    break

                writer.writerow(parts)
                saved += 1

                if now - last_report >= 1.0:
                    recorded = elapsed - args.warmup
                    print(f"Recorded {saved:6d} rows ({recorded:6.1f} s)", end="\r")
                    output_file.flush()
                    last_report = now

    except KeyboardInterrupt:
        print("\nCapture stopped by user.")
    except OSError as exc:
        print(f"\nSerial or output error: {exc}", file=sys.stderr)
        return 2

    print(f"\nSaved {saved} rows to {output}")
    if malformed:
        print(f"Ignored {malformed} malformed IMU rows.")
    if saved < 2:
        print("Too few samples were captured for statistics.", file=sys.stderr)
        return 1

    print(f"Next: python3 analyze.py {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
