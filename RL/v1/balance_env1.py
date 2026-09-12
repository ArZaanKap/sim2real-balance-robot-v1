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


        self.action_hist = np.zeros(len(self.action_space) * self.len_action_hist) # 2 * 3  # or deque?
        self.obs_hist = np.zeros(len(self.observation_space) * (1+self.len_obs_hist))

        self.max_torque = 0.43 # Nm - derived
        self.physics_substeps = 5 # 500hz physics vs 100hz control
        self.step_count = 0
        self.max_steps = 2000

        self.fall_angle = 0.6   # rad - TERMINAL

        self.prev_action = None


    def reset(self):
        super().reset(seed=seed)

        mujoco.mj_resetData(self.model, self.data)

        # init dr mass, dr obs0,

        # randomise max torque at start of eps
        self.max_torque = np.random.uniform(0.35, 0.45) # right place?

        # init state DR
        pitch0 = np.random.uniform(-1.0, 1.0) # rad
        pitch_rate0 = np.random.uniform(-0.5, 0.5) # rad/s

        # qpos = [x, y, z, qw, qx, qy, qz] 
        self.data.qpos[3:7] = [np.cos(pitch0/2), 0.0, np.sin(pitch0/2), 0.0]  # amend quaternion to apply init rotation offset [3:7]

        # qvel = [vx, vy, vz, wx, wy, wz, wheelL_w, wheelR_w]
        self.data.qvel[4] = pitch_rate0

        mujoco.mj_forward(self.model, self.data)
        self.step_count = 0

        return self._get_obs(), {}
        

    def _get_obs(self):
        quat = self.data.sensor("imu_quat").data    # rot stored as quarternion - must extract theta
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        R = R.reshape(3,3)
        
        pitch = np.arctan2(R[0,2], R[2,2]) + np.random.uniform(-0.1, 0.1) # x = atan2(sin(x)/cos(x)) -> 3d y rot matrix
        
        pitch_rate = self.data.sensor("imu_gyro").data[1] + np.random.uniform(-0.3, 0.3)  # (wx, wy, wz) -> gyros give angular velocity in each axis
        
        # read gt wheel vel with noise (not available on robot - differentiate encoder readings then filter instead)
        wl = self.data.sensor("wheel_left_vel").data[0] + np.random.uniform(-0.3, 0.3)  # (angular vel) [0] cos length 1 vector
        wr = self.data.sensor("wheel_right_vel").data[0] + np.random.uniform(-0.3, 0.3)

        return np.array([pitch, pitch_rate, wl, wr], dtype=np.float32)


    def step(self, action):

        # torque = policy output clip to [-1, 1] * max torque
        self.data.ctrl[:] = np.clip(action, -1.0, 1.0) * self.max_torque # whats data.ctrl look like - 

        # advance physics and obs with action
        for i in range(self.physics_substeps):
            mujoco.mj_step(self.model, self.data)

        self.step_count += 1
        obs = self._get_obs()
        pitch, pitch_rate, wl, wr = obs

        # ---- REWARDS ---- #
        upright = 1.0 - (pitch/self.fall_angle)**2

        action_pen = -0.001 * np.sum(np.square(action)) # penalise large torques

        action_rate_pen = 0.0
        if prev_action is not None:
            action_rate_pen = -0.005 * np.sum(np.square(action - prev_action))
        
        self.prev_action = action.copy()

        reward = upright + action_pen + action_rate_pen

        terminated = bool(abs(pitch) > self.fall_angle)
        truncated = bool(self.step_count >= self.max_steps)

        return obs, float(reward), terminated, truncated, {}