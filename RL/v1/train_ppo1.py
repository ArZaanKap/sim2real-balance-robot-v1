from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.callbacks import EvalCallback, BaseCallback

import os

from balance_env1 import BalanceEnv

# ---- change per run (one place) ----
RUN = 8
MODEL_PATH = "../models/model1.xml"
SAVE_DIR = "trained_policies1"
# ------------------------------------

make_env = lambda: BalanceEnv(model_path=MODEL_PATH)   # reused for train + eval envs
best_dir = os.path.join(SAVE_DIR, f"best{RUN}")        # EvalCallback writes best_model.zip here


# saves the CURRENT vecnorm stats whenever EvalCallback hits a new best,
# so best_model.zip is paired with the obs-normalisation from the SAME instant
# (EvalCallback alone only saves the policy, not the stats -> mismatch at play/sim2real)
class SaveVecnorm(BaseCallback):
    def __init__(self, save_path):
        super().__init__()
        self.save_path = save_path
    def _on_step(self):
        self.model.get_vec_normalize_env().save(self.save_path)
        return True


check_env(make_env())

v_env = make_vec_env(make_env, n_envs=8)
v_env = VecNormalize(v_env, norm_obs=True, norm_reward=False)

# separate frozen env for evaluation (training=False -> apply stats, don't update them).
# EvalCallback auto-syncs obs stats from v_env before each eval since both are VecNormalize.
eval_env = make_vec_env(make_env, n_envs=1)
eval_env = VecNormalize(eval_env, norm_obs=True, norm_reward=False, training=False)


model = PPO("MlpPolicy", v_env, verbose=1, device="cpu",
            tensorboard_log="./tb",
            n_steps=256,   #2048
            batch_size=64,
            n_epochs=10,
            gamma=0.995,
            gae_lambda=0.95,
            ent_coef=0.0,
            vf_coef=0.5, # value loss weight ratio vs ..?
            learning_rate=3e-4,
            max_grad_norm=0.5, # how to check how much this is effecting?
            normalize_advantage=True,
            seed=0,
        ) #explain all?


os.makedirs(SAVE_DIR, exist_ok=True)

# best-so-far checkpoint by mean eval reward. eval_freq is PER-ENV -> 5000*8 = 40k global steps.
# on a new best it saves best{RUN}/best_model.zip AND best{RUN}/vecnorm.pkl (via callback_on_new_best).
eval_cb = EvalCallback(
    eval_env,
    best_model_save_path=best_dir,
    eval_freq=5000,
    n_eval_episodes=10,
    deterministic=True,
    callback_on_new_best=SaveVecnorm(os.path.join(best_dir, "vecnorm.pkl")),
)

model.learn(total_timesteps=600_000, callback=eval_cb) # 300k

# final-weights save kept too, so you can compare best-vs-final
model.save(os.path.join(SAVE_DIR, f"ppo_balance{RUN}"))
v_env.save(os.path.join(SAVE_DIR, f"vecnorm{RUN}.pkl"))
