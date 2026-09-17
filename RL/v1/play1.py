import time, numpy as np, mujoco, mujoco.viewer
from balance_env1 import BalanceEnv
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

model = PPO.load("trained_policies1/ppo_balance4")

# load the obs mean/var saved during training (throwaway venv just to hold the stats)
vecnorm = VecNormalize.load("trained_policies1/vecnorm4.pkl",
                            DummyVecEnv([lambda: BalanceEnv(model_path="../models/model1.xml")]))
vecnorm.training = False        # freeze running stats - don't update at eval

env = BalanceEnv(model_path="../models/model1.xml")
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
