from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from balance_env0 import BalanceEnv
import os


env = BalanceEnv()
check_env(env) # ?

model = PPO("MlpPolicy", env, verbose=1, device="cpu",
            tensorboard_log="./tb",
            n_steps=2048,
            gamma=0.99,
            learning_rate=3e-4,
            seed=0,
            
        ) #explain all?


model.learn(total_timesteps=300_000)

save_dir = "trained_policies"
os.makedirs(save_dir, exist_ok=True)
model.save(os.path.join(save_dir,"ppo_balance0"))