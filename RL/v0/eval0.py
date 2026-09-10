import numpy as np
from balance_env0 import BalanceEnv
from stable_baselines3 import PPO



def run_episode(model, env, seed):
    obs, _ = env.reset(seed=seed)          # fixed seed = same perturbation for every policy
    start_xy = env.data.qpos[0:2].copy()   # .copy()! qpos is a live buffer
    prev_xy  = start_xy.copy()
    prev_a   = None

    speeds, yaws, pitches = [], [], []
    actions, action_rates = [], []
    path_len, max_exc = 0.0, 0.0
    steps = 0

    while True:
        action, _ = model.predict(obs, deterministic=True)
        obs, _, term, trunc, _ = env.step(action)
        steps += 1

        xy = env.data.qpos[0:2].copy()
        speeds.append(np.linalg.norm(env.data.qvel[0:2])) # base speed
        yaws.append(env.data.qvel[5]) # yaw rate
        pitches.append(obs[0])
        actions.append(np.abs(action).mean()) # mean |action| this step

        if prev_a is not None: # skip 1st step (no prev)
            action_rates.append(np.sum((action - prev_a)**2)) # Δaction²
        prev_a = action.copy()

        path_len += np.linalg.norm(xy - prev_xy) # cumulative distance travelled
        max_exc = max(max_exc, np.linalg.norm(xy - start_xy))
        prev_xy = xy

        if term or trunc:
            break

    return dict(steps=steps,
                rms_pitch = np.sqrt(np.mean(np.square(pitches))),
                rms_speed = np.sqrt(np.mean(np.square(speeds))),
                rms_yaw   = np.sqrt(np.mean(np.square(yaws))),
                mean_act  = np.mean(actions),
                rms_arate = np.sqrt(np.mean(action_rates)) if action_rates else 0.0,
                path_len  = path_len,
                max_exc   = max_exc)


def evaluate(policy_name, seeds):
    env = BalanceEnv()
    model = PPO.load(f"trained_policies/{policy_name}")
    eps = [run_episode(model, env, s) for s in seeds]
    return {k: np.mean([e[k] for e in eps]) for k in eps[0]} # mean across episodes


if __name__ == "__main__":
    seeds = list(range(20)) # 20 shared episodes each
    policies = ["ppo_balance2", "ppo_balance7", "ppo_balance8", "ppo_balance9"]  # ones to compare
    cols = ["steps","rms_pitch","rms_speed","rms_yaw","mean_act","rms_arate","path_len","max_exc"]

    rows = {p: evaluate(p, seeds) for p in policies}
    print(f"{'policy':16s}" + "".join(f"{c:>11s}" for c in cols))
    for p, m in rows.items():
        print(f"{p:16s}" + "".join(f"{m[c]:11.3f}" for c in cols))