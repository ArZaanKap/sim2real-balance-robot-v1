import numpy as np
import gymnasium as gym
import mujoco
from gymnasium.spaces import Box


class BalanceEnv(gym.env):

    def __init__(self, model_path):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        self.len_action_hist = 3
        self.len_obs_hist = 3

        # 2 floats [-1,1] (left, right)
        self.action_space = Box(-1.0, 1.0, shape=(2,), dtype=np.float32)

        # pitch from imu (on board filter), pitch rate (imu gyro raw y), wheel_w: differentiate encoder readings + 1st order filter (in sim read raw vel + DR)
        # [(pitch, pitch_rate, wl_w, wr_w), action_hist, obs_hist] 
        self.observation_space = Box(-np.inf, np.inf, 
            shape=(4*(self.len_obs_hist+1) + 2*(self.len_action_hist),),
            dtype=np.float32)


        self.action_hist = []
        self.obs_hist = []

        self.max_torque = 0.43 # Nm - derived
        self.physics_substeps = 5 # 500hz physics vs 100hz control
        self.step_count = 0

        self.fall_angle = 0.6   # rad - TERMINAL


    def reset(self):
        super().reset(seed=seed)

        mujoco.mj_resetData(self.model, self.data)

        # init dr mass, dr obs0,

        # init state DR
        pitch0 = np.random.uniform(-1.0, 1.0) # rad
        pitch_rate0 = np.random.uniform(-0.5, 0.5) # rad/s

        # qpos = [x, y, z, qw, qx, qy, qz] 
        self.data.qpos[3:7] =   # amend quaternion to apply init rotation offset [3:7]


        # qvel = [vx, vy, vz, wx, wy, wz, wheelL_w, wheelR_w]
        self.data.qvel[4] = pitch_rate0

