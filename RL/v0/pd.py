import time, numpy as np, mujoco, mujoco.viewer
from train_balance0 import BalanceEnv

env = BalanceEnv()
obs, _ = env.reset(seed=1)
kp, kd = 40.0, 1.0  # +sign PD on pitch (balances-but-drifts)

with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
    while viewer.is_running():
        t0 = time.time()
        pitch, p_rate, wl, wr = obs
        u = kp * pitch + kd * p_rate     # same torque to both wheels -> straight
        
        obs, r, term, trunc, _ = env.step(np.array([u, u], dtype=np.float32))
        viewer.sync()
        if term or trunc:               # fell or timed out -> new episode
            obs, _ = env.reset()

        # real-time pace: one env.step = n_substeps * timestep = 0.01 s (100 Hz)
        dt = env.n_substeps * env.model.opt.timestep - (time.time() - t0)
        if dt > 0:
            time.sleep(dt)