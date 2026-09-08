import numpy as np
import gymnasium as gym
import mujoco
from gymnasium.spaces import Box


class BalanceEnv(gym.Env):
    def __init__(self):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path("../models/model0.xml")
        self.data = mujoco.MjData(self.model)

        # out = [-1, 1] for each of 2 wheels
        self.action_space = Box(-1.0, 1.0, shape=(2,), dtype=np.float32)

        # obs = [pitch, pitch_rate, wheelL_w, wheelR_w]
        self.observation_space = Box(-np.inf, np.inf, shape=(4,), dtype=np.float32)

        self.max_torque = 0.43
        self.n_substeps = 5     # physics 5x control policy
        self.max_steps = 2000   # episode cap (20s at 100Hz)
        self.step_count = 0

        self.fall_angle = 0.6 # rad 
        self.start_pos = None

    def reset(self, *, seed=None, options=None): # * means keyword only
        super().reset(seed=seed)

        mujoco.mj_resetData(self.model, self.data)
        self.start_pos = self.data.qpos[0:3].copy() # start (x,y,z)

        # randomise starting pitch
        pitch0 = self.np_random.uniform(-0.3, 0.3) # rad (~16deg)

        # randomise starting pitch_rate - TODO LATER
        pitch_rate0 = self.np_random.uniform(-0.1, 0.1) # rad/s
        
        # qpos = [x, y, z, qw, qx, qy, qz] 3 to 6 is rotation quaternion
        self.data.qpos[3:7] = [np.cos(pitch0/2), 0.0, np.sin(pitch0/2), 0.0] # standard quarternion rotation formula TODO revise
        
        # qvel = [vx, vy, vz, wx, wy, wz, wheelL_w, wheelR_w]
        self.data.qvel[4] = pitch_rate0

        mujoco.mj_forward(self.model, self.data) # recompute sensor data for 1st obs
        self.step_count = 0 # reset

        return self._get_obs(), {} #?

    
    def _get_obs(self):
        quat = self.data.sensor("imu_quat").data
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        R = R.reshape(3,3)
        pitch = np.arctan2(R[0,2], R[2,2]) # x = atan2(sin(x)/cos(x)) -> 3d y rot matrix
        pitch_rate = self.data.sensor("imu_gyro").data[1] # (wx, wy, wz) -> gyros give angular velocity in each axis
        wl = self.data.sensor("wheel_left_vel").data[0] # (angular vel) [0] cos length 1 vector
        wr = self.data.sensor("wheel_right_vel").data[0]

        return np.array([pitch, pitch_rate, wl, wr], dtype=np.float32)

    
    def step(self, action):

        # torque = policy output clip to [-1, 1] * max torque
        self.data.ctrl[:] = np.clip(action, -1.0, 1.0) * self.max_torque # [:]? all set to result of the *? so 2 vals in ctrl

        # advance physics by 5 steps for 1 control step (500Hz vs 100Hz)
        for _ in range(self.n_substeps):
            mujoco.mj_step(self.model, self.data)

        self.step_count += 1
        obs = self._get_obs()
        pitch, pitch_rate, wl, wr = obs

        #-----REWARDS----#
        # quadratic -> could test exp(-k*pitch^2) TODO compare training graphs
        upright = 1.0 - (pitch / self.fall_angle)**2 # 1 upright, 0 at fall_angle (linear gradient - bigger errors have higher gradient - stronger pull upwards); reward can go neg (fine)
        
        # try vel vs pos?
        #drift_pen = -0.1 * np.sum((self.data.qpos[0:3] - self.start_pos)**2)
        #drift_pen = -0.5 * np.sum(self.data.qvel[0:3]**2) # penalise linear vel of robot base in x y z (z=0 anyway) to attack drift
        # see if effort pen enough to make it not run away?

        effort_pen = -0.002 * np.sum(np.square(action))  # neg reward

        reward = upright + effort_pen #+ drift_pen

        # --- episode end --- #
        terminated = bool(abs(pitch) > self.fall_angle) # fell over
        truncated = bool(self.step_count >= self.max_steps) # max episode len reached

        return obs, float(reward), terminated, truncated, {}
