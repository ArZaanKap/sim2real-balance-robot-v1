import time, numpy as np, mujoco, mujoco.viewer
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from balance_env1 import BalanceEnv

# ---- pick what to play (one place) ----
RUN = 10
MODEL_PATH = "../models/model1.xml"
SAVE_DIR = "trained_policies1"
USE_BEST = True   # True -> best{RUN}/ checkpoint (save-best); False -> final ppo_balance{RUN}
# ---------------------------------------

make_env = lambda: BalanceEnv(model_path=MODEL_PATH)

if USE_BEST:
    policy_path, vecnorm_path = f"{SAVE_DIR}/best{RUN}/best_model", f"{SAVE_DIR}/best{RUN}/vecnorm.pkl"
else:
    policy_path, vecnorm_path = f"{SAVE_DIR}/ppo_balance{RUN}", f"{SAVE_DIR}/vecnorm{RUN}.pkl"

model = PPO.load(policy_path)

# load the obs mean/var saved during training (throwaway venv just to hold the stats)
vecnorm = VecNormalize.load(vecnorm_path, DummyVecEnv([make_env]))
vecnorm.training = False        # freeze running stats - don't update at eval

env = make_env()
obs, _ = env.reset()

with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
    while viewer.is_running():
        t0 = time.time()
        pitch, p_rate, wl, wr = obs[:4]

        norm_obs = vecnorm.normalize_obs(obs)           # SAME (obs-mean)/std used in training
        action, _ = model.predict(norm_obs, deterministic=True)

        obs, r, term, trunc, _ = env.step(action)
        viewer.sync()
        if term or trunc:               # fell or timed out -> new episode
            obs, _ = env.reset()

        # real-time pace: one env.step = physics_substeps * timestep = 0.01 s (100 Hz)
        dt = env.physics_substeps * env.model.opt.timestep - (time.time() - t0)
        if dt > 0:
            time.sleep(dt)
