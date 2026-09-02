# live_plot.py — watch a BAM sweep as it runs: velocity + position vs time, live.
#
# Reads the sweep_logger's rows straight off the wire and scrolls a rolling window, so
# you can SEE the staircase / spin-down / deadband happening (and spot a stalled motor,
# a flipped encoder, or a noisy velocity signal) before you ever fit anything.
#
# Row format (from sweep_logger.c):  BAM, t_ms, phase, duty(-255..255), pos_counts, vel_rad_s
#
# TWO WAYS TO FEED IT (pick whichever the capture uses):
#   A) let it open the port itself:
#        python live_plot.py --port /dev/ttyUSB0
#   B) pipe the same stream you capture with (no port lock contention):
#        stty -F /dev/ttyUSB0 115200 raw && cat /dev/ttyUSB0 | python live_plot.py
#
# Ctrl-C to stop. Depends on: matplotlib, and pyserial only if you use --port.

import argparse
import collections
import sys
import threading

import matplotlib.pyplot as plt


def line_source(port, baud):
    """Yield text lines from a serial port (--port) or from piped stdin."""
    if port:
        import serial   # only needed for --port
        with serial.Serial(port, baud, timeout=1) as ser:
            while True:
                raw = ser.readline()
                if raw:
                    yield raw.decode("ascii", "replace")
    else:
        for line in sys.stdin:
            yield line


def parse(line):
    """One 'BAM,...' data row -> (t_ms, phase, duty, counts, vel), or None to skip."""
    line = line.strip()
    if not line.startswith("BAM,"):
        return None
    p = [c.strip() for c in line.split(",")]   # tolerate "BAM, 100, ..." or "BAM,100,..."
    if len(p) < 6 or p[2] in ("HEADER", "DONE", "end") or p[1] == "MARK":
        return None
    try:
        return int(p[1]), p[2], int(p[3]), int(p[4]), float(p[5])
    except (ValueError, IndexError):
        return None


def reader(src, store, maxlen):
    """Background thread: parse rows into shared deques (append is atomic in CPython)."""
    for line in src:
        row = parse(line)
        if row is None:
            continue
        t_ms, phase, duty, counts, vel = row
        if store["t0"] is None:
            store["t0"] = t_ms
        store["t"].append((t_ms - store["t0"]) / 1000.0)
        store["vel"].append(vel)
        store["pos"].append(counts / store["cpr"])   # revolutions
        store["phase"] = phase
    store["done"] = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="serial port to open (omit to read piped stdin)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--counts-per-rev", type=float, default=1976.0, help="for the position axis [rev]")
    ap.add_argument("--window", type=float, default=20.0, help="rolling window shown [s]")
    ap.add_argument("--maxlen", type=int, default=20000, help="max samples kept in memory")
    args = ap.parse_args()

    store = {
        "t": collections.deque(maxlen=args.maxlen),
        "vel": collections.deque(maxlen=args.maxlen),
        "pos": collections.deque(maxlen=args.maxlen),
        "t0": None, "phase": "", "cpr": args.counts_per_rev, "done": False,
    }

    src = line_source(args.port, args.baud)
    threading.Thread(target=reader, args=(src, store, args.maxlen), daemon=True).start()

    plt.ion()
    fig, (ax_v, ax_p) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    (ln_v,) = ax_v.plot([], [], color="C0", lw=1.4)
    (ln_p,) = ax_p.plot([], [], color="C2", lw=1.4)
    ax_v.set_ylabel("velocity [rad/s]"); ax_v.grid(alpha=0.3)
    ax_p.set_ylabel("position [rev]"); ax_p.set_xlabel("time [s]"); ax_p.grid(alpha=0.3)
    ax_v.axhline(0, color="0.7", lw=0.8)

    try:
        while plt.fignum_exists(fig.number):
            t = list(store["t"]); vel = list(store["vel"]); pos = list(store["pos"])
            if t:
                n = min(len(t), len(vel), len(pos))   # a row may be mid-append
                t, vel, pos = t[:n], vel[:n], pos[:n]
                ln_v.set_data(t, vel)
                ln_p.set_data(t, pos)
                hi = t[-1]
                lo = max(0.0, hi - args.window)
                for ax in (ax_v, ax_p):
                    ax.set_xlim(lo, hi if hi > lo else lo + 1)
                    ax.relim(); ax.autoscale_view(scalex=False)
                ax_v.set_title(f"phase: {store['phase']}   |   {n} samples   t={hi:.1f}s")
            fig.canvas.draw_idle()
            plt.pause(0.05)
            if store["done"] and not plt.fignum_exists(fig.number):
                break
    except KeyboardInterrupt:
        pass
    print("\nlive_plot stopped.")


if __name__ == "__main__":
    main()
