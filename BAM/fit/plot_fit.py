# plot_fit.py — VISUALLY check a BAM fit: overlay the fitted model's velocity on the
# measured encoder velocity, one panel per sweep, with residuals underneath.
#
# No hardware. This replays the recorded voltage through the fitted model exactly the
# way the fitter scored it (Simulator.rollout_log, simulate_control=False) and draws
# sim-vs-measured. If the fit is good the two lines sit on top of each other and the
# residuals hug zero; a systematic gap points at a wrong friction/electrical term.
#
# USAGE
#   python plot_fit.py --logs logs/ --params ga25-370_motor1_params.json
#   python plot_fit.py --logs logs/ --params ga25-370_motor1_params.json --save fit.png
#
# Depends only on: bam, numpy, matplotlib (same env as the fitter, plus matplotlib).

import argparse
import json

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from bam.simulate import Simulator

from freeshaft import FreeShaft, GA25Actuator
from fit_freeshaft import build_model, load_logs


def model_from_params(params: dict, vin: float):
    """Rebuild the fitted model: models[name]() + GA25Actuator, values from params.json.

    We reuse build_model so the friction bounds / pinned nuisance params match the fit,
    then just stamp in every value the JSON recorded.
    """
    model = build_model(params["model"], vin=vin, kt=params.get("kt"), R=params.get("R"))
    for name, p in model.get_parameters().items():
        if name in params:
            p.value = float(params[name])
    return model


def plot(logs, model, title, save=None):
    sim = Simulator(model)
    n = len(logs)
    # One column per sweep; velocity on top, residual (sim - measured) on the bottom.
    fig, axes = plt.subplots(
        2, n, figsize=(5.2 * n, 6), sharex="col",
        gridspec_kw={"height_ratios": [3, 1]}, squeeze=False,
    )

    for col, log in enumerate(logs):
        name = log.get("name", f"sweep {col + 1}")
        dt = log["dt"]
        _, sim_vel, control = sim.rollout_log(log, simulate_control=False)
        sim_vel = np.asarray(sim_vel)
        meas = np.array([e["speed"] for e in log["entries"]])
        t = np.arange(len(meas)) * dt
        resid = sim_vel - meas
        mae = float(np.mean(np.abs(resid)))

        ax_v, ax_r = axes[0][col], axes[1][col]

        # Faint voltage trace (twin axis) so the velocity response is readable in context.
        ax_c = ax_v.twinx()
        ax_c.plot(t, control, color="0.8", lw=1.0, zorder=1)
        ax_c.set_ylabel("volts", color="0.6", fontsize=9)
        ax_c.tick_params(axis="y", labelcolor="0.6", labelsize=8)

        ax_v.plot(t, meas, color="C0", lw=1.6, label="measured", zorder=3)
        ax_v.plot(t, sim_vel, color="C1", lw=1.4, ls="--", label="fitted", zorder=4)
        ax_v.set_title(f"{name}   (MAE {mae:.3f} rad/s)")
        ax_v.set_ylabel("velocity [rad/s]")
        ax_v.grid(alpha=0.3)
        ax_v.set_zorder(ax_c.get_zorder() + 1)   # velocity lines above the voltage trace
        ax_v.patch.set_visible(False)
        if col == 0:
            ax_v.legend(loc="best", fontsize=9)

        ax_r.axhline(0, color="0.6", lw=0.8)
        ax_r.plot(t, resid, color="C3", lw=1.0)
        ax_r.set_ylabel("resid")
        ax_r.set_xlabel("time [s]")
        ax_r.grid(alpha=0.3)

    fig.suptitle(title, fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    if save:
        fig.savefig(save, dpi=120)
        print(f"wrote {save}")
    else:
        plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", required=True, help="dir of processed BAM json logs (from csv_to_bam.py)")
    ap.add_argument("--params", required=True, help="fitted params.json (from fit_freeshaft.py)")
    ap.add_argument("--vin", type=float, default=None,
                    help="bus voltage [V]; default = the vin recorded in the logs")
    ap.add_argument("--save", type=str, default=None,
                    help="write a PNG instead of opening a window (use when headless)")
    args = ap.parse_args()

    if args.save:
        matplotlib.use("Agg")   # no display needed

    logs = load_logs(args.logs)
    # Tag each log with its filename so panels are labelled by sweep.
    import glob
    import os
    for log, path in zip(logs, sorted(glob.glob(f"{args.logs}/*.json"))):
        log["name"] = os.path.splitext(os.path.basename(path))[0]

    with open(args.params) as f:
        params = json.load(f)
    vin = args.vin if args.vin is not None else logs[0].get("vin", 12.0)

    model = model_from_params(params, vin)
    plot(logs, model, title=f"{args.params}  (model {params['model']}, vin {vin} V)",
         save=args.save)


if __name__ == "__main__":
    main()
