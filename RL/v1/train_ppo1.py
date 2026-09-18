from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize

import os

from balance_env1 import BalanceEnv



check_env(BalanceEnv(model_path="../models/model1.xml"))

v_env = make_vec_env(lambda: 
                        BalanceEnv(model_path="../models/model1.xml"),
                        n_envs=8,
                    )  

v_env = VecNormalize(v_env, norm_obs=True, norm_reward=False)


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


model.learn(total_timesteps=600_000) # 300k

save_dir = "trained_policies1"
os.makedirs(save_dir, exist_ok=True)
model.save(os.path.join(save_dir,"ppo_balance6"))
v_env.save(os.path.join(save_dir, "vecnorm6.pkl"))