# csv_to_bam.py — turn the ESP32 sweep_logger's run.csv into BAM-format JSON logs.
#
# sweep_logger.c prints rows:  BAM, t_ms, phase, duty(-255..255), pos_counts, vel_rad_s
# BAM's fitter wants JSON logs:  {dt, mass, arm_mass, length, kp, vin,
#                                 entries:[{position[rad], speed[rad/s], control[V],
#                                           torque_enable}]}
#
# The mapping:
#   position      = pos_counts / counts_per_rev * 2*pi        [rad]
#   speed         = vel_rad_s                                 [rad/s] (already computed on-MCU)
#   control       = duty/255 * vin                            [V]  (BAM works in volts)
#   torque_enable = (duty != 0)                               <-- CRITICAL, see note below
#
# torque_enable=False whenever duty==0: both DRV8871 inputs low = COAST (outputs high-Z),
# so the motor terminals are open, there is NO back-EMF braking, only mechanical friction.
# If we left torque_enable=True with 0 V, BAM's tau = -kt^2*w/R would wrongly brake the
# coast-down and corrupt the cleanest friction signal we have. So duty==0 => not powered.
#
# We emit three logs mirroring the three sweeps so the fitter sees clean segments:
#   staircase.json  (phase "stair")
#   spindown.json   (phases "spinup" + "coast")
#   deadband.json   (phase "deadband")
#
# USAGE:  python csv_to_bam.py run.csv logs/ --counts-per-rev 1976 --vin 11.97

import argparse
import json
import os
import math

# Which CSV phase labels belong to which output log (in capture order).
GROUPS = {
    "staircase": ["stair"],
    "spindown": ["spinup", "coast"],
    "deadband": ["deadband"],
}


def parse_rows(path):
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line.startswith("BAM,"):
                continue
            parts = line.split(",")
            phase = parts[2]
            # skip non-data markers: HEADER, DONE, MARK(breakaway)
            if phase in ("HEADER", "DONE", "end") or parts[1] in ("MARK",):
                continue
            try:
                rows.append({
                    "t_ms": int(parts[1]),
                    "phase": phase,
                    "duty": int(parts[3]),
                    "counts": int(parts[4]),
                    "vel": float(parts[5]),
                })
            except (ValueError, IndexError):
                continue  # tolerate a truncated first/last line
    return rows


def estimate_dt(rows):
    ts = [r["t_ms"] for r in rows]
    diffs = [b - a for a, b in zip(ts, ts[1:]) if 0 < b - a < 1000]
    if not diffs:
        return 0.01
    diffs.sort()
    return diffs[len(diffs) // 2] / 1000.0  # median, ms -> s


def to_entry(r, counts_per_rev, vin):
    return {
        "position": r["counts"] / counts_per_rev * 2.0 * math.pi,
        "speed": r["vel"],
        "control": r["duty"] / 255.0 * vin,
        "torque_enable": r["duty"] != 0,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("outdir")
    ap.add_argument("--counts-per-rev", type=float, required=True,
                    help="output-shaft counts/rev (same value calibrated in sweep_logger.c)")
    ap.add_argument("--vin", type=float, default=12.0, help="bus voltage at duty=255 [V]")
    ap.add_argument("--stride", type=int, default=1,
                    help="keep every Nth sample (downsample for a faster fit; dt scales up). "
                         "Friction/decel curves are smooth, so 3-4 is plenty.")
    args = ap.parse_args()

    rows = parse_rows(args.csv)
    if not rows:
        raise SystemExit("no BAM data rows found in " + args.csv)
    dt = estimate_dt(rows) * args.stride
    if args.stride > 1:
        rows = rows[::args.stride]

    # Encoder A/B may be wired opposite to the motor drive direction, so +duty gives
    # -velocity. The model assumes +voltage -> +velocity, so a flipped encoder makes the
    # fit fight the data (MAE ~= 2x the speed). Detect it (sum of duty*vel over moving
    # samples) and flip the encoder sign (position + velocity) to agree with the drive.
    corr = sum(r["duty"] * r["vel"] for r in rows)
    flip = corr < 0
    if flip:
        print("  encoder sign is INVERTED vs drive (+duty -> -vel) -> flipping to agree")
        for r in rows:
            r["counts"] = -r["counts"]
            r["vel"] = -r["vel"]
    os.makedirs(args.outdir, exist_ok=True)

    for name, phases in GROUPS.items():
        entries = [to_entry(r, args.counts_per_rev, args.vin)
                   for r in rows if r["phase"] in phases]
        if not entries:
            print(f"  (no rows for {name}, skipping)")
            continue
        log = {"dt": dt, "mass": 0.0, "arm_mass": 0.0, "length": 0.0,
               "kp": 0.0, "vin": args.vin, "entries": entries}
        out = os.path.join(args.outdir, name + ".json")
        json.dump(log, open(out, "w"))
        print(f"  wrote {out}: {len(entries)} samples")

    print(f"dt = {dt*1000:.1f} ms  |  next: python fit_freeshaft.py --logs {args.outdir} "
          f"--model m1 --kt 0.24 --R 6.65")


if __name__ == "__main__":
    main()
