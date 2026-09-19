"""Deterministic, headless policy evaluation for the v2 balance environment.

Examples (run from RL/v2):
    python eval2.py                         # quick check: best run 1, seeds 0..19
    python eval2.py --episodes 300          # fuller fixed-seed audit
    python eval2.py --run 1 --final         # evaluate final rather than best
    python eval2.py --run 1 2 --episodes 300  # compare on identical seeds
    python eval2.py --seed-start 20000      # use a different fixed seed set
"""

import argparse
import os

import numpy as np
from stable_baselines3 import PPO

from balance_env2 import BalanceEnv


MODEL_PATH = "../models/model1.xml"
SAVE_DIR = "trained_policies2"


def load_policy(run, use_best, make_env):
    if use_best:
        policy_path = os.path.join(SAVE_DIR, f"best{run}", "best_model")
        label = f"best{run}"
    else:
        policy_path = os.path.join(SAVE_DIR, f"ppo_balance{run}")
        label = f"final{run}"

    model = PPO.load(policy_path, device="cpu")
    return label, model


def run_episode(model, env, seed):
    obs, _ = env.reset(seed=seed)
    start_x = float(env.data.qpos[0])
    max_excursion = 0.0
    previous_action = None

    pitch_sq_sum = 0.0
    action_delta_sq_sum = 0.0
    action_delta_count = 0
    settled_action_delta_sq_sum = 0.0
    settled_action_delta_count = 0
    action_abs_sum = 0.0
    saturation_count = 0
    steps = 0

    while True:
        # env already emits normalised obs (fixed scales) -> feed straight in
        action, _ = model.predict(obs, deterministic=True)
        action = np.asarray(action, dtype=np.float64).reshape(-1)

        obs, _, terminated, truncated, _ = env.step(action)
        steps += 1

        pitch = env.get_pitch()  # ground truth, not the noisy policy observation
        pitch_sq_sum += float(pitch * pitch)
        action_abs_sum += float(np.abs(action).mean())
        saturation_count += int(np.any(np.abs(action) >= 0.95))

        if previous_action is not None:
            action_delta_sq = float(np.mean(np.square(action - previous_action)))
            action_delta_sq_sum += action_delta_sq
            action_delta_count += 1
            if steps > 100:  # exclude the first second of initial recovery
                settled_action_delta_sq_sum += action_delta_sq
                settled_action_delta_count += 1
        previous_action = action.copy()

        max_excursion = max(max_excursion, abs(float(env.data.qpos[0]) - start_x))
        if terminated or truncated:
            break

    return {
        "steps": steps,
        "success": int(truncated and not terminated),
        "pitch_sq_sum": pitch_sq_sum,
        "action_delta_sq_sum": action_delta_sq_sum,
        "action_delta_count": action_delta_count,
        "settled_action_delta_sq_sum": settled_action_delta_sq_sum,
        "settled_action_delta_count": settled_action_delta_count,
        "action_abs_sum": action_abs_sum,
        "saturation_count": saturation_count,
        "end_displacement": abs(float(env.data.qpos[0]) - start_x),
        "max_excursion": max_excursion,
    }


def evaluate(run, use_best, seeds):
    make_env = lambda: BalanceEnv(model_path=MODEL_PATH)
    label, model = load_policy(run, use_best, make_env)
    env = make_env()
    episodes = [run_episode(model, env, seed) for seed in seeds]

    steps = np.asarray([ep["steps"] for ep in episodes])
    total_steps = int(steps.sum())
    total_deltas = sum(ep["action_delta_count"] for ep in episodes)
    total_settled_deltas = sum(ep["settled_action_delta_count"] for ep in episodes)

    metrics = {
        "episodes": len(episodes),
        "mean_survival_s": float(steps.mean() * 0.01),
        "median_survival_s": float(np.median(steps) * 0.01),
        "success_pct": float(100.0 * np.mean([ep["success"] for ep in episodes])),
        "fell_under_1s_pct": float(100.0 * np.mean(steps < 100)),
        "pitch_rms_deg": float(np.degrees(np.sqrt(sum(ep["pitch_sq_sum"] for ep in episodes) / total_steps))),
        "mean_abs_action": float(sum(ep["action_abs_sum"] for ep in episodes) / total_steps),
        "action_delta_rms": float(np.sqrt(sum(ep["action_delta_sq_sum"] for ep in episodes) / total_deltas)),
        "settled_action_delta_rms": (
            float(np.sqrt(sum(ep["settled_action_delta_sq_sum"] for ep in episodes) / total_settled_deltas))
            if total_settled_deltas
            else float("nan")
        ),
        "saturation_pct": float(100.0 * sum(ep["saturation_count"] for ep in episodes) / total_steps),
        "mean_abs_end_x_m": float(np.mean([ep["end_displacement"] for ep in episodes])),
        "mean_max_excursion_m": float(np.mean([ep["max_excursion"] for ep in episodes])),
    }
    env.close()
    return label, metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", type=int, nargs="+", default=[1])
    parser.add_argument("--final", action="store_true", help="Use final weights instead of best checkpoint")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed-start", type=int, default=0)
    args = parser.parse_args()

    for index, run in enumerate(args.run):
        seeds = range(args.seed_start, args.seed_start + args.episodes)
        label, metrics = evaluate(run, not args.final, seeds)

        if index:
            print()
        print(f"policy: {label}")
        print(f"seeds:  {args.seed_start}..{args.seed_start + args.episodes - 1}")
        print(f"episodes:                  {metrics['episodes']:8d}")
        print(f"mean survival:             {metrics['mean_survival_s']:8.2f} s")
        print(f"median survival:           {metrics['median_survival_s']:8.2f} s")
        print(f"20-second success:         {metrics['success_pct']:8.1f} %")
        print(f"fell within 1 second:      {metrics['fell_under_1s_pct']:8.1f} %")
        print(f"pitch RMS:                 {metrics['pitch_rms_deg']:8.2f} deg")
        print(f"mean |action|:             {metrics['mean_abs_action']:8.3f}")
        print(f"command-change RMS:        {metrics['action_delta_rms']:8.3f}")
        print(f"change RMS after 1 second: {metrics['settled_action_delta_rms']:8.3f}")
        print(f"action saturation (>=.95): {metrics['saturation_pct']:8.1f} %")
        print(f"mean absolute end x:       {metrics['mean_abs_end_x_m']:8.3f} m")
        print(f"mean max x excursion:      {metrics['mean_max_excursion_m']:8.3f} m")


if __name__ == "__main__":
    main()
