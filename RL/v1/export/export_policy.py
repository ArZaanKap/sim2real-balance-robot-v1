"""Export a trained v1 PPO balancer to a C header + parity fixtures.

What it emits (into export/run{RUN}/):
  policy.h   - DATA ONLY: VecNorm affine + MLP weights/biases as float32 arrays.
               You write the C forward pass; this just gives it the numbers.
  fixtures.h - N (raw_obs -> expected_action) pairs from real rollouts, so your
               C can printf its action over serial and diff against ground truth.

It also runs an in-process PARITY CHECK: a from-scratch numpy reimplementation of the exact
inference path (VecNorm normalize+clip -> tanh MLP -> action_net -> clip) vs SB3's own
predict(). If those don't agree to ~1e-6 the export is wrong and it refuses to write.

Run from RL/v1:   python export/export_policy.py
"""
import os, sys, warnings
import numpy as np
warnings.filterwarnings("ignore")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # RL/v1 on path

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from balance_env1 import BalanceEnv

# ---- what to export ----
RUN = 13
MODEL_PATH = "../models/model1.xml"
SAVE_DIR = "trained_policies1"
N_FIXTURES = 12          # how many (obs->action) test vectors to bake into the C header
PARITY_SAMPLES = 500     # rollout steps used to stress the parity check
TOL = 2e-6               # np-vs-SB3 max abs action diff we require (both run float32)
# ------------------------

HERE = os.path.dirname(os.path.abspath(__file__))
mk = lambda: BalanceEnv(model_path=MODEL_PATH)


def load():
    policy_path = os.path.join(SAVE_DIR, f"best{RUN}", "best_model")
    vecnorm_path = os.path.join(SAVE_DIR, f"best{RUN}", "vecnorm.pkl")
    model = PPO.load(policy_path, device="cpu")
    vn = VecNormalize.load(vecnorm_path, DummyVecEnv([mk]))
    vn.training = False
    return model, vn


def extract(model, vn):
    """Pull every number the C side needs, as float32."""
    pol = model.policy
    p = pol.mlp_extractor.policy_net
    d = {
        # VecNorm affine (obs -> (obs-mean)/sqrt(var+eps), then clip to +/-clip)
        "obs_mean": vn.obs_rms.mean.astype(np.float32),
        "obs_var":  vn.obs_rms.var.astype(np.float32),
        "eps":      np.float32(vn.epsilon),
        "clip":     np.float32(vn.clip_obs),
        # MLP: layer0 (19->64) tanh, layer2 (64->64) tanh, action_net (64->1)
        "w0": p[0].weight.detach().numpy().astype(np.float32),  # (64,19)
        "b0": p[0].bias.detach().numpy().astype(np.float32),    # (64,)
        "w1": p[2].weight.detach().numpy().astype(np.float32),  # (64,64)
        "b1": p[2].bias.detach().numpy().astype(np.float32),    # (64,)
        "wa": pol.action_net.weight.detach().numpy().astype(np.float32),  # (1,64)
        "ba": pol.action_net.bias.detach().numpy().astype(np.float32),    # (1,)
    }
    d["obs_dim"] = d["w0"].shape[1]
    d["act_dim"] = d["wa"].shape[0]
    d["hid"] = d["w0"].shape[0]
    return d


def np_forward(raw_obs, d):
    """From-scratch reference: this is EXACTLY what your C must reproduce."""
    x = raw_obs.astype(np.float32)
    # VecNorm: normalize then clip
    x = (x - d["obs_mean"]) / np.sqrt(d["obs_var"] + d["eps"], dtype=np.float32)
    x = np.clip(x, -d["clip"], d["clip"]).astype(np.float32)
    # MLP
    h = np.tanh(d["w0"] @ x + d["b0"]).astype(np.float32)
    h = np.tanh(d["w1"] @ h + d["b1"]).astype(np.float32)
    a = (d["wa"] @ h + d["ba"]).astype(np.float32)
    # SB3 predict() clips the action to the Box bounds [-1,1]
    return np.clip(a, -1.0, 1.0).astype(np.float32)


def collect_rollout(model, vn, n):
    """Real (raw_obs, sb3_action) pairs - representative, not gaussian noise."""
    env = mk()
    obs, _ = env.reset(seed=0)
    raws, acts = [], []
    for _ in range(n):
        norm = vn.normalize_obs(obs)
        act, _ = model.predict(norm, deterministic=True)
        raws.append(obs.copy().astype(np.float32))
        acts.append(np.asarray(act, dtype=np.float32).reshape(-1))
        obs, _, term, trunc, _ = env.step(act)
        if term or trunc:
            obs, _ = env.reset()
    return np.array(raws), np.array(acts)


# ---------- C emitters ----------
def cf(v):
    """A valid C float literal: '.9g' then guarantee a '.'/'e' before the f suffix."""
    s = f"{float(v):.9g}"
    if not any(c in s for c in ".eEnN"):  # 0 -> 0.0, 10 -> 10.0 (nan/inf keep as-is)
        s += ".0"
    return s + "f"


def carr(name, a):
    a = np.asarray(a, dtype=np.float32).ravel()
    body = ", ".join(cf(v) for v in a)
    return f"static const float {name}[{a.size}] = {{ {body} }};\n"


def cmat(name, M):
    M = np.asarray(M, dtype=np.float32)
    rows = ",\n  ".join("{ " + ", ".join(cf(v) for v in row) + " }" for row in M)
    return f"static const float {name}[{M.shape[0]}][{M.shape[1]}] = {{\n  {rows}\n}};\n"


def write_header(d, outdir):
    path = os.path.join(outdir, "policy.h")
    with open(path, "w") as f:
        f.write(f"// AUTO-GENERATED from best{RUN} by export_policy.py -- DATA ONLY, do not edit.\n")
        f.write(f"// Inference order the C must implement:\n")
        f.write(f"//   x = clip((obs - OBS_MEAN)/sqrt(OBS_VAR+EPS), -CLIP, CLIP)\n")
        f.write(f"//   h = tanh(W0 @ x + B0);  h = tanh(W1 @ h + B1)\n")
        f.write(f"//   a = clip(WA @ h + BA, -1, 1)\n")
        f.write("#pragma once\n\n")
        f.write(f"#define OBS_DIM {d['obs_dim']}\n#define HID {d['hid']}\n#define ACT_DIM {d['act_dim']}\n")
        f.write(f"static const float POLICY_EPS  = {cf(d['eps'])};\n")
        f.write(f"static const float POLICY_CLIP = {cf(d['clip'])};\n\n")
        f.write(carr("OBS_MEAN", d["obs_mean"]))
        f.write(carr("OBS_VAR",  d["obs_var"]) + "\n")
        f.write(cmat("W0", d["w0"]) + carr("B0", d["b0"]) + "\n")
        f.write(cmat("W1", d["w1"]) + carr("B1", d["b1"]) + "\n")
        f.write(cmat("WA", d["wa"]) + carr("BA", d["ba"]))
    return path


def write_fixtures(raws, acts, outdir):
    path = os.path.join(outdir, "fixtures.h")
    idx = np.linspace(0, len(raws) - 1, N_FIXTURES).astype(int)
    with open(path, "w") as f:
        f.write(f"// AUTO-GENERATED test vectors from best{RUN}. Feed each obs to your C forward\n")
        f.write(f"// pass; the result must match act to within ~1e-4 (float32 rounding).\n")
        f.write("#pragma once\n\n")
        f.write(f"#define N_FIX {len(idx)}\n")
        f.write(f"#define FIX_OBS_DIM {raws.shape[1]}\n\n")
        f.write(cmat("FIX_OBS", raws[idx]))
        f.write(carr("FIX_ACT", acts[idx].ravel()))
    return path


def main():
    model, vn = load()
    d = extract(model, vn)
    raws, sb3_acts = collect_rollout(model, vn, PARITY_SAMPLES)

    # PARITY: my numpy path vs SB3's own predict, on the same raw obs
    mine = np.array([np_forward(o, d) for o in raws]).reshape(sb3_acts.shape)
    err = np.max(np.abs(mine - sb3_acts))
    print(f"run best{RUN}: obs_dim={d['obs_dim']} hid={d['hid']} act_dim={d['act_dim']}")
    print(f"parity samples={len(raws)}  max |np - SB3| = {err:.2e}  (tol {TOL:.0e})")
    if err > TOL:
        raise SystemExit(f"PARITY FAILED ({err:.2e} > {TOL:.0e}) -- export is wrong, not writing.")

    outdir = os.path.join(HERE, f"run{RUN}")   # one folder per policy, e.g. export/run13/
    os.makedirs(outdir, exist_ok=True)
    hp = write_header(d, outdir)
    fp = write_fixtures(raws, sb3_acts, outdir)
    print(f"parity OK -> run{RUN}/{os.path.basename(hp)} + run{RUN}/{os.path.basename(fp)}")
    print("action range over rollout:", f"[{sb3_acts.min():.3f}, {sb3_acts.max():.3f}]")


if __name__ == "__main__":
    main()
