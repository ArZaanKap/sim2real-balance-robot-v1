import numpy as np
import gymnasium as gym
import mujoco
from gymnasium.spaces import Box


class BalanceEnv(gym.Env):

    def __init__(self, model_path):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # store model xml data before DR applied
        self.nominal_body_mass = self.model.body_mass.copy()
        self.nominal_body_inertia = self.model.body_inertia.copy()

        self.len_action_hist = 3
        self.len_obs_hist = 3

        self.core_obs_len = 4 # pitch, pitch_rate, wl_w, wr_w

        # 2 floats [-1,1] (left, right)
        self.action_space = Box(-1.0, 1.0, shape=(2,), dtype=np.float32)

        # pitch from imu (on board filter), pitch rate (imu gyro raw y), wheel_w: differentiate encoder readings + 1st order filter (in sim read raw vel + DR)
        # [(pitch, pitch_rate, wl_w, wr_w), action_hist, obs_hist] 
        self.observation_space = Box(-np.inf, np.inf, 
            shape=(self.core_obs_len*(self.len_obs_hist+1) + self.action_space.shape[0]*(self.len_action_hist),),
            dtype=np.float32)


        self.action_hist = np.zeros(self.action_space.shape[0] * self.len_action_hist) # 2 * 3  # store prev N actions
        self.obs_hist = np.zeros(self.core_obs_len * self.len_obs_hist) # store prev N observations

        self.max_torque = 0.43 # Nm - derived
        self.physics_substeps = 5 # 500hz physics vs 100hz control
        self.step_count = 0
        self.max_steps = 2000

        self.fall_angle = 0.6   # rad - TERMINAL

        self.prev_action = None


    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        mujoco.mj_resetData(self.model, self.data)

        # reset histories
        self.action_hist = np.zeros(self.action_space.shape[0] * self.len_action_hist) # 2 * 3  # or deque?
        self.obs_hist = np.zeros(self.core_obs_len * self.len_obs_hist)

        # mass/inertia DR
        for i in range(1,self.model.nbody): # skip body 0 (world - has no mass)
            f = np.random.uniform(0.8,1.2)  # must be same factor for mass & inertia else physically wrong?
            self.model.body_mass[i] = self.nominal_body_mass * f
            self.model.body_inertia[i] = self.nominal_body_inertia * f



        # randomise max torque at start of eps
        self.max_torque = np.random.uniform(0.35, 0.45) # right place?

        # init state DR
        pitch0 = np.random.uniform(-1.0, 1.0) # rad
        pitch_rate0 = np.random.uniform(-0.5, 0.5) # rad/s

        # qpos = [x, y, z, qw, qx, qy, qz] 
        self.data.qpos[3:7] = [np.cos(pitch0/2), 0.0, np.sin(pitch0/2), 0.0]  # amend quaternion to apply init rotation offset [3:7]

        # qvel = [vx, vy, vz, wx, wy, wz, wheelL_w, wheelR_w]
        self.data.qvel[4] = pitch_rate0

        # UPDATE sim with changes above
        mujoco.mj_forward(self.model, self.data)
        self.step_count = 0

        # has to match step()?
        full_obs = np.array([*self._get_obs(), *self.action_hist, *self.obs_hist])
        return full_obs, {}
    
    # helper to get pitch
    def get_pitch(self):
        quat = self.data.sensor("imu_quat").data    # rot stored as quarternion - must extract theta
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        R = R.reshape(3,3)
        return np.arctan2(R[0,2], R[2,2])  # x = atan2(sin(x)/cos(x)) -> 3d y rot matrix

    def _get_obs(self):
        
        pitch = self.get_pitch() + np.random.uniform(-0.1, 0.1)
        pitch_rate = self.data.sensor("imu_gyro").data[1] + np.random.uniform(-0.3, 0.3)  # (wx, wy, wz) -> gyros give angular velocity in each axis
        
        # read gt wheel vel with noise (not available on robot - differentiate encoder readings then filter instead)
        wl = self.data.sensor("wheel_left_vel").data[0] + np.random.uniform(-0.3, 0.3)  # (angular vel) [0] cos length 1 vector
        wr = self.data.sensor("wheel_right_vel").data[0] + np.random.uniform(-0.3, 0.3)

        return np.array([pitch, pitch_rate, wl, wr], dtype=np.float32) # return core obs (without histories)


    def step(self, action):

        # torque = policy output clip to [-1, 1] * max torque
        self.data.ctrl[:] = np.clip(action, -1.0, 1.0) * self.max_torque # whats data.ctrl look like - 

        # advance physics with action input and construct new obs
        for i in range(self.physics_substeps):
            mujoco.mj_step(self.model, self.data)

        # update action history here - front is most recent
        self.action_hist = np.roll(self.action_hist, self.action_space.shape[0])  # queue of prev actions
        self.action_hist[0:self.action_space.shape[0]] = action

        self.step_count += 1
        obs = self._get_obs()
        #pitch, pitch_rate, wl, wr = obs
        full_obs = np.array([*obs, *self.action_hist, *self.obs_hist])  # full obs to return

        # update obs hist here
        self.obs_hist = np.roll(self.obs_hist, self.core_obs_len)
        self.obs_hist[0:self.core_obs_len] = np.array([*obs]) # expand obs

        # ---- REWARDS ---- #
        # use GT vals from sim - not noisy vals from obs

        gt_pitch = self.get_pitch()

        upright = 1.0 - (gt_pitch/self.fall_angle)**2
        action_pen = -0.001 * np.sum(np.square(action)) # penalise large torques

        action_rate_pen = 0.0
        if self.prev_action is not None:
            action_rate_pen = -0.005 * np.sum(np.square(action - self.prev_action))
        
        self.prev_action = action.copy()

        reward = upright + action_pen + action_rate_pen

        terminated = bool(abs(gt_pitch) > self.fall_angle)
        truncated = bool(self.step_count >= self.max_steps)

        return full_obs, float(reward), terminated, truncated, {}