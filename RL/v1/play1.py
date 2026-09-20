import time, numpy as np, mujoco, mujoco.viewer
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from balance_env1 import BalanceEnv

# ---- pick what to play (one place) ----
RUN = 11
MODEL_PATH = "../models/model1.xml"
SAVE_DIR = "trained_policies1"
USE_BEST = True   # True -> best{RUN}/ checkpoint (save-best); False -> final ppo_balance{RUN}
# ---------------------------------------

# ---- disturbance push ("someone shoves the top of the neck") ----
PUSH_FORCE = 0.5    # N - current shove magnitude (change live with UP/DOWN)
                    # 808g robot, shove lands 316mm up: 0.5N~0.45rad/s kick, 1N~0.9, 5N~4.5 (unrecoverable).
                    # trained only on +/-0.15 rad/s init & no mid-episode pushes, so keep it in ~0.3-1N.
PUSH_STEP  = 0.25   # N - how much UP/DOWN nudges the magnitude
PUSH_STEPS = 8      # control steps to HOLD the shove (~80 ms at 100 Hz) -> impulse, not wind
# GLFW arrow keycodes: RIGHT/LEFT = shove +x/-x (tips pitch), UP/DOWN = magnitude +/-
GLFW_RIGHT, GLFW_LEFT, GLFW_UP, GLFW_DOWN = 262, 263, 265, 264
# -----------------------------------------------------------------

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
env.max_steps = 3000            # 30 s in play (default 2000 = 20 s); training horizon untouched
obs, _ = env.reset()

# body we shove; xfrc_applied acts at its CoM in WORLD frame
neck_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "neck")

# shared state the key_callback pokes; the main loop consumes it
push = {"steps": 0, "dir": 0.0, "force": PUSH_FORCE}

def on_key(keycode):
    if keycode == GLFW_RIGHT:
        push["steps"], push["dir"] = PUSH_STEPS, +1.0
    elif keycode == GLFW_LEFT:
        push["steps"], push["dir"] = PUSH_STEPS, -1.0
    elif keycode == GLFW_UP:
        push["force"] += PUSH_STEP
        print(f"push force = {push['force']:.1f} N")
    elif keycode == GLFW_DOWN:
        push["force"] = max(0.0, push["force"] - PUSH_STEP)
        print(f"push force = {push['force']:.1f} N")

print(f"keys: RIGHT/LEFT = shove neck +x/-x,  UP/DOWN = force +/-  (start {PUSH_FORCE:.1f} N)")

with mujoco.viewer.launch_passive(env.model, env.data,
                                  key_callback=on_key,
                                  show_left_ui=False, show_right_ui=False) as viewer:
    while viewer.is_running():
        t0 = time.time()
        pitch, p_rate, wl, wr = obs[:4]

        # apply / hold / release the shove (persists across mj_step until we clear it)
        if push["steps"] > 0:
            env.data.xfrc_applied[neck_id, 0] = push["dir"] * push["force"]
            push["steps"] -= 1
        else:
            env.data.xfrc_applied[neck_id, :3] = 0.0

        norm_obs = vecnorm.normalize_obs(obs)           # SAME (obs-mean)/std used in training
        action, _ = model.predict(norm_obs, deterministic=True)

        obs, r, term, trunc, _ = env.step(action)
        viewer.sync()
        if term or trunc:               # fell or timed out -> new episode
            push["steps"] = 0           # drop any in-flight shove so it doesn't bleed over
            obs, _ = env.reset()        # mj_resetData zeros xfrc_applied

        # real-time pace: one env.step = physics_substeps * timestep = 0.01 s (100 Hz)
        dt = env.physics_substeps * env.model.opt.timestep - (time.time() - t0)
        if dt > 0:
            time.sleep(dt)
