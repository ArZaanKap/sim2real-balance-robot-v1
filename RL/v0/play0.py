import time, numpy as np, mujoco, mujoco.viewer
from balance_env0 import BalanceEnv
from stable_baselines3 import PPO

model = PPO.load("trained_policies/ppo_balance12")
env = BalanceEnv()
obs, _ = env.reset()

with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
    while viewer.is_running():
        t0 = time.time()
        pitch, p_rate, wl, wr = obs

        action, _ = model.predict(obs, deterministic=True)
        
        obs, r, term, trunc, _ = env.step(action)
        viewer.sync()
        if term or trunc:               # fell or timed out -> new episode
            obs, _ = env.reset()

        # real-time pace: one env.step = n_substeps * timestep = 0.01 s (100 Hz)
        dt = env.n_substeps * env.model.opt.timestep - (time.time() - t0)
        if dt > 0:
            time.sleep(dt)