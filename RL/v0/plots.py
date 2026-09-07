"""plots.py — PPO curves straight from TensorBoard event files (no server).
  python plots.py                                  # newest run in ./tb
  python plots.py tb/PPO_1 tb/PPO_3 --labels no-BAM BAM --smooth 0.9
"""
import glob, os, argparse
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

# the 6 metrics actually worth watching, and why each earns its panel:
PANELS = [
    ("rollout/ep_len_mean",      "Episode length (->cap = learned not to fall)"),
    ("rollout/ep_rew_mean",      "Episode reward (task performance)"),
    ("train/explained_variance", "Explained variance (critic quality, ->1)"),
    ("train/approx_kl",          "Approx KL (update size / stability)"),
    ("train/entropy_loss",       "Entropy (exploration level)"),
    ("train/std",                "Action std (exploration collapse?)"),
]

def ema(y, a):                      # smoothing: RL curves are noisy
    if a <= 0: return y
    out, m = [], y[0]
    for v in y: m = a*m + (1-a)*v; out.append(m)
    return out

def load(run):
    ea = EventAccumulator(run); ea.Reload()
    tags = ea.Tags().get("scalars", [])
    return {t: ([e.step for e in ea.Scalars(t)], [e.value for e in ea.Scalars(t)]) for t in tags}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--labels", nargs="*")
    ap.add_argument("--smooth", type=float, default=0.0)   # try 0.9
    ap.add_argument("--out", default="curves.png")
    a = ap.parse_args()

    runs = a.runs or [max(glob.glob("tb/PPO_*"), key=os.path.getmtime)]
    labels = a.labels or [os.path.basename(r) for r in runs]

    fig, axs = plt.subplots(2, 3, figsize=(16, 8))
    for run, lab in zip(runs, labels):
        d = load(run)
        for ax, (tag, title) in zip(axs.flat, PANELS):
            ax.set_title(title, fontsize=10); ax.grid(alpha=0.3); ax.set_xlabel("timesteps")
            if tag in d:
                x, y = d[tag]; ax.plot(x, ema(y, a.smooth), lw=1.6, label=lab)
    axs.flat[0].legend(fontsize=9)
    fig.tight_layout(); fig.savefig(a.out, dpi=120); print("saved", a.out)

if __name__ == "__main__":
    main()
