from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback

import os

from balance_env2 import BalanceEnv

# ---- change per run (one place) ----
RUN = 4
MODEL_PATH = "../models/model1.xml"
SAVE_DIR = "trained_policies2"
# ------------------------------------

make_env = lambda: BalanceEnv(model_path=MODEL_PATH)   # reused for train + eval envs
best_dir = os.path.join(SAVE_DIR, f"best{RUN}")        # EvalCallback writes best_model.zip here


# NO VecNormalize now: the env emits already-normalised obs (fixed hardcoded
# scales, mean 0). So there are no running stats to save/pair -> best_model.zip
# is fully self-contained, and play/eval/firmware all use the SAME fixed divide.
check_env(make_env())

v_env = make_vec_env(make_env, n_envs=8, seed=0)

# separate eval env on a different fixed seed set (frozen, never trains)
eval_env = make_vec_env(make_env, n_envs=1, seed=10_000)


model = PPO("MlpPolicy", v_env, verbose=1, device="cpu",
            tensorboard_log="./tb",
            n_steps=256,   #2048
            batch_size=64,
            n_epochs=10,
            gamma=0.995,
            gae_lambda=0.95,
            ent_coef=0.0,  # 0.005
            vf_coef=0.5, # value loss weight ratio vs ..?
            learning_rate=3e-4,
            max_grad_norm=0.5, # keep tight; 1.0 permits bigger steps -> late instability
            normalize_advantage=True,
            seed=0,
        ) #explain all?


os.makedirs(SAVE_DIR, exist_ok=True)

# best-so-far checkpoint by mean eval reward. eval_freq is PER-ENV -> 5000*8 = 40k global steps.
# on a new best it saves best{RUN}/best_model.zip (no vecnorm to pair now).
eval_cb = EvalCallback(
    eval_env,
    best_model_save_path=best_dir,
    eval_freq=5000,
    n_eval_episodes=50,
    deterministic=True,
)

model.learn(total_timesteps=400_000, callback=eval_cb) # 300k

# final-weights save kept too, so you can compare best-vs-final
model.save(os.path.join(SAVE_DIR, f"ppo_balance{RUN}"))
