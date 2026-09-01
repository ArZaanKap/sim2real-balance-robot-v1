# fit_freeshaft.py — Phase 2 of the BAM plan: fit a GA25-370 friction model to our
# open-loop bench sweep, reusing BAM's real model math + MuJoCo/Warp export.
#
# WHY NOT `python -m bam.fit`?
# ----------------------------
# bam.fit is hardwired for smart servos on a pendulum: its objective replays a
# position-GOAL trajectory through the servo's P-controller (simulate_control=True)
# and scores POSITION error against a gravity load. Our data is open-loop voltage on
# a free-spinning shaft, and for a wheel that never returns to a setpoint, position
# drifts unboundedly — matching VELOCITY is the right objective. So we drive BAM's
# own Simulator with simulate_control=False (it replays our recorded voltage straight
# into tau = kt*V/R - kt^2*w/R) and optimize the friction parameters ourselves with
# SciPy. The fitted object is a real BAM Model, so Phase 3 (to_mujoco / mjlab Warp
# export) works unchanged.
#
# USAGE
#   python fit_freeshaft.py --selftest                 # no hardware: prove the fitter
#   python fit_freeshaft.py --logs logs/ --model m1    # fit real captured data
#   python fit_freeshaft.py --logs logs/ --model m1 --kt 0.24 --R 6.65  # pin electrical (Motor #1)
#
# Depends only on: bam, numpy, scipy  (no optuna/wandb/mujoco needed for the fit).

import argparse
import glob
import json

import numpy as np
from scipy.optimize import differential_evolution

from bam.model import models
from bam.simulate import Simulator

from freeshaft import FreeShaft, GA25Actuator


# --- Build a free-shaft, voltage-driven BAM model ------------------------------------
def build_model(model_name: str, vin: float, kt: float | None, R: float | None):
    """models[name]() + our GA25Actuator on a FreeShaft testbench.

    If kt / R are given they are PINNED (optimize=False) at the hand-measured values;
    otherwise they stay free. The friction terms are always left free.
    """
    model = models[model_name]()
    model.set_actuator(GA25Actuator(FreeShaft, vin=vin))

    # BAM's default friction ceilings are sized for small serial servos. Our GA25 at
    # the OUTPUT shaft (post-38:1) is far more friction-dominated (it breaks away at
    # ~55% voltage), so widen the static-friction bound or the optimizer jams on it.
    if "friction_base" in model.get_parameters():
        model.friction_base.max = 0.6
    if "friction_stribeck" in model.get_parameters():
        model.friction_stribeck.max = 0.6

    # Pin the rig nuisance params. Both are for BAM's servo+pendulum setup and are
    # meaningless here: command_delay only acts when simulate_control=True (a goal
    # trajectory we don't have), and q_offset only shifts absolute position, which a
    # velocity-matching objective ignores. Freeing them just wastes optimizer budget.
    for nuisance in ("q_offset", "command_delay"):
        p = model.get_parameters().get(nuisance)
        if p is not None:
            p.value, p.optimize = 0.0, False

    if kt is not None:
        model.kt.value, model.kt.optimize = kt, False
    if R is not None:
        model.R.value, model.R.optimize = R, False
    return model


# --- Objective: does the model reproduce the recorded VELOCITY? ----------------------
def velocity_mae(model, logs) -> float:
    """Mean-abs velocity error over all logs, replaying recorded voltage."""
    sim = Simulator(model)
    total, n = 0.0, 0
    for log in logs:
        _, vel, _ = sim.rollout_log(log, simulate_control=False)
        vel = np.asarray(vel)
        rec = np.array([e["speed"] for e in log["entries"]])
        total += float(np.mean(np.abs(vel - rec)))
        n += 1
    return total / max(n, 1)


def fit(model, logs, seed: int = 0, maxiter: int = 100):
    """Optimize every Parameter with optimize=True against velocity_mae (SciPy DE)."""
    params = model.get_parameters()
    names = [k for k, p in params.items() if p.optimize]
    bounds = [(params[k].min, params[k].max) for k in names]

    def loss(x):
        for k, v in zip(names, x):
            params[k].value = float(v)
        return velocity_mae(model, logs)

    res = differential_evolution(
        loss, bounds, seed=seed, maxiter=maxiter, tol=1e-8, polish=True
    )
    for k, v in zip(names, res.x):
        params[k].value = float(v)
    return {k: params[k].value for k in names}, res.fun


# --- I/O -----------------------------------------------------------------------------
def load_logs(logdir: str):
    logs = []
    for f in sorted(glob.glob(f"{logdir}/*.json")):
        with open(f) as fh:
            logs.append(json.load(fh))
    if not logs:
        raise SystemExit(f"no *.json logs in {logdir} (run csv_to_bam.py first)")
    return logs


def save_params(model, model_name, out):
    data = {k: p.value for k, p in model.get_parameters().items()}
    data["model"] = model_name
    json.dump(data, open(out, "w"), indent=2)
    print(f"wrote {out}")


# --- Self-test: synth data from KNOWN params, then check the fitter recovers them -----
def make_synthetic_logs(true_model, dt=0.01):
    """Roll the TRUE model forward on a control schedule and record it as BAM logs.

    Two logs mirroring the real sweeps: a staircase (steps of voltage, powered) and a
    spin-up-then-coast (torque_enable flips False -> pure friction decel).
    """
    sim = Simulator(true_model)
    rng = np.random.default_rng(1)

    def rollout(schedule):
        # schedule: list of (volts, torque_enable, n_steps)
        sim.reset(0.0, 0.0)
        # Give the actuator a log so its testbench/vin are set (values unused by FreeShaft).
        sim.model.actuator.load_log({"kp": 0.0, "vin": true_model.actuator.vin})
        entries = []
        for volts, te, n in schedule:
            for _ in range(n):
                # record state BEFORE stepping (matches rollout_log's convention)
                noise = rng.normal(0.0, 0.02)  # ~encoder/velocity noise [rad/s]
                entries.append({
                    "position": float(sim.q),
                    "speed": float(sim.dq + noise),
                    "control": float(volts),
                    "torque_enable": bool(te),
                })
                sim.step(volts, te, dt)
        return {"dt": dt, "mass": 0.0, "arm_mass": 0.0, "length": 0.0,
                "kp": 0.0, "vin": true_model.actuator.vin, "entries": entries}

    v = true_model.actuator.vin
    staircase = rollout([(0.25 * v, True, 60), (0.5 * v, True, 60),
                         (0.75 * v, True, 60), (1.0 * v, True, 60)])
    spindown = rollout([(0.9 * v, True, 120), (0.0, False, 200)])
    return [staircase, spindown]


def selftest():
    print("=== self-test: recover known friction params from synthetic data ===")
    # Ground-truth model (m3, load-dependent) with hand-chosen params.
    truth = build_model("m3", vin=12.0, kt=None, R=None)
    true_vals = {"kt": 0.021, "R": 4.2, "armature": 3e-4,
                 "friction_base": 0.012, "friction_viscous": 0.05}
    for k, val in true_vals.items():
        truth.get_parameters()[k].value = val
    if "load_friction_base" in truth.get_parameters():
        true_vals["load_friction_base"] = 0.08
        truth.get_parameters()["load_friction_base"].value = 0.08

    logs = make_synthetic_logs(truth)

    # Fit from scratch, but PIN kt & R (Plan A: measured by hand), fit friction+armature.
    model = build_model("m3", vin=12.0, kt=true_vals["kt"], R=true_vals["R"])
    fitted, loss = fit(model, logs, maxiter=120)

    print(f"final velocity MAE: {loss:.4f} rad/s")
    print(f"{'param':22s} {'true':>10s} {'fitted':>10s}")
    ok = True
    # Terms that ARE identifiable from free-shaft (no external load) data — the ones
    # that matter for balancing. We assert on these.
    for k in ["armature", "friction_base", "friction_viscous"]:
        t, f = true_vals[k], fitted[k]
        rel = abs(f - t) / (abs(t) + 1e-9)
        flag = "" if rel < 0.25 else "  <-- off"
        if rel >= 0.25:
            ok = False
        print(f"{k:22s} {t:10.5f} {f:10.5f}{flag}")
    # m3's load-dependence coefficient multiplies gearbox_torque = |motor_torque| on a
    # free shaft, which correlates with the base terms — so it is WEAKLY identifiable
    # without a real load. Reported, not asserted. (Add a known load to pin it, or just
    # accept it: balancing cares about static + viscous, both nailed above.)
    if "load_friction_base" in fitted:
        print(f"{'load_friction_base':22s} {true_vals['load_friction_base']:10.5f} "
              f"{fitted['load_friction_base']:10.5f}  (weakly identifiable, informational)")
    print("RESULT:", "PASS" if (ok and loss < 0.05) else "CHECK", "\n")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--logs", type=str, help="dir of processed BAM json logs")
    ap.add_argument("--model", type=str, default="m1",
                    help="m1..m6 (default m1 = Coulomb+viscous; right for free-shaft/balancing. "
                         "m3's load term is unidentifiable without an external load — see README)")
    ap.add_argument("--vin", type=float, default=12.0)
    ap.add_argument("--kt", type=float, default=None, help="pin kt [Nm/A] (Plan A)")
    ap.add_argument("--R", type=float, default=None, help="pin R [Ohm] (Plan A)")
    ap.add_argument("--out", type=str, default="params.json")
    ap.add_argument("--maxiter", type=int, default=100,
                    help="SciPy differential-evolution iterations (raise for a finer fit)")
    args = ap.parse_args()

    if args.selftest:
        raise SystemExit(0 if selftest() else 1)

    if not args.logs:
        ap.error("give --logs DIR (or --selftest)")
    logs = load_logs(args.logs)
    model = build_model(args.model, args.vin, args.kt, args.R)
    fitted, loss = fit(model, logs, maxiter=args.maxiter)
    print(f"velocity MAE: {loss:.4f} rad/s")
    for k, v in fitted.items():
        print(f"  {k:22s} {v:.6f}")
    save_params(model, args.model, args.out)


if __name__ == "__main__":
    main()
