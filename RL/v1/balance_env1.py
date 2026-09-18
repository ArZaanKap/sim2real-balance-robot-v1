import numpy as np
import gymnasium as gym
import mujoco
from gymnasium.spaces import Box

# 1 rad ~= 57deg

class BalanceEnv(gym.Env):

    def __init__(self, model_path):
        super().__init__()

        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # store model xml data before DR applied
        self.nominal_body_mass = self.model.body_mass.copy()
        self.nominal_body_inertia = self.model.body_inertia.copy()
        self.nominal_gain = self.model.actuator_gainprm.copy()
        self.nominal_bias = self.model.actuator_biasprm.copy()
        self.nominal_frictionloss = self.model.dof_frictionloss.copy()
        self.nominal_geom_friction = self.model.geom_friction.copy()
        self.nominal_armature = self.model.dof_armature.copy()

        # very hard to understand these...?
        wheel_bodies = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, n) for n in ("wheel_left", "wheel_right")]
        self.wheel_geom_ids = [g for g in range(self.model.ngeom) if self.model.geom_bodyid[g] in wheel_bodies]
        self.wheel_dof_ids = [self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in ("wheel_left_joint", "wheel_right_joint")]

        self.len_action_hist = 3
        self.len_obs_hist = 3

        self.core_obs_len = 4 # pitch, pitch_rate, wl_w, wr_w
        self.num_actions = 2

        # 2 floats [-1,1] (left, right)
        self.action_space = Box(-1.0, 1.0, shape=(self.num_actions,), dtype=np.float32)

        # pitch from imu (on board filter), pitch rate (imu gyro raw y), wheel_w: differentiate encoder readings + 1st order filter (in sim read raw vel + DR)
        # [(pitch, pitch_rate, wl_w, wr_w), action_hist, obs_hist] 
        self.observation_space = Box(-np.inf, np.inf, 
            shape=(self.core_obs_len*(self.len_obs_hist+1) + self.num_actions*(self.len_action_hist),),
            dtype=np.float32)

        # histories for obs
        self.action_hist = np.zeros(self.num_actions * self.len_action_hist) # 2 * 3  # store prev N actions
        self.obs_hist = np.zeros(self.core_obs_len * self.len_obs_hist) # store prev N observations

        self.physics_substeps = 5 # 500hz physics vs 100hz control
        self.step_count = 0
        self.max_steps = 2000

        self.fall_angle = 0.6   # rad - TERMINAL

        self.prev_action = None


    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        mujoco.mj_resetData(self.model, self.data)

        # reset histories
        self.action_hist = np.zeros(self.num_actions * self.len_action_hist, dtype=np.float32) # 2 * 3  # or deque?
        self.obs_hist = np.zeros(self.core_obs_len * self.len_obs_hist, dtype=np.float32)

        # reset pre action cos new episode starts
        self.prev_action = None

        # mass/inertia DR
        for i in range(1,self.model.nbody): # skip body 0 (world - has no mass)
            f = self.np_random.uniform(0.8,1.2)  # must be same factor for mass & inertia else physically wrong?
            self.model.body_mass[i] = self.nominal_body_mass[i] * f
            self.model.body_inertia[i] = self.nominal_body_inertia[i] * f
        
        # MOTOR ELECTRICAL DR - need explaining math
        kt_R_f = self.np_random.uniform(0.85, 1.15) # kt/R - scale stall and droop together
        vbus_f = self.np_random.uniform(0.90, 1.05) # bus voltage scales stall only??
        self.model.actuator_gainprm[:, 0] = self.nominal_gain[:,0] * kt_R_f * vbus_f
        self.model.actuator_biasprm[:, 2] = self.nominal_bias[:, 2] * kt_R_f

        # TIRE/FLOOR FRICTION DR
        mu_f = self.np_random.uniform(0.6, 1.1) # lower mu means more slippery
        for g in self.wheel_geom_ids:
            self.model.geom_friction[g, 0] = self.nominal_geom_friction[g,0] * mu_f

        # frictionloss DEADBAND & armature DR
        for d in self.wheel_dof_ids:
            self.model.dof_frictionloss[d] = self.nominal_frictionloss[d] * self.np_random.uniform(0.75, 1.25)
            self.model.dof_armature[d] = self.nominal_armature[d] * self.np_random.uniform(0.9, 1.1) # tight

        # buffers for latency DR (action & obs MAYBE)
        # action_buf stores raw actions
        self.action_buf = np.zeros(self.num_actions * self.np_random.integers(1,4)) # random action delay in control steps() -> so 10ms - 40ms
        

        # INIT STATE DR - should add noise here too? else 1st clean?
        pitch0 = self.np_random.uniform(-0.3, 0.3) # rad
        pitch_rate0 = self.np_random.uniform(-0.3, 0.3) # rad/s

        # qpos = [x, y, z, qw, qx, qy, qz] 
        self.data.qpos[3:7] = [np.cos(pitch0/2), 0.0, np.sin(pitch0/2), 0.0]  # amend quaternion to apply init rotation offset [3:7]

        # qvel = [vx, vy, vz, wx, wy, wz, wheelL_w, wheelR_w]
        self.data.qvel[4] = pitch_rate0


        # UPDATE sim with changes above
        mujoco.mj_forward(self.model, self.data)
        self.step_count = 0

        # has to match step()?
        full_obs = np.array([*self._get_obs(), *self.action_hist, *self.obs_hist], dtype=np.float32)
        return full_obs, {}
    
    # helper to get pitch
    def get_pitch(self):
        quat = self.data.sensor("imu_quat").data    # rot stored as quarternion - must extract theta
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, quat)
        R = R.reshape(3,3)
        return np.arctan2(R[0,2], R[2,2])  # x = atan2(sin(x)/cos(x)) -> 3d y rot matrix

    def _get_obs(self):
        
        pitch = self.get_pitch() + self.np_random.uniform(-0.04, 0.04)
        pitch_rate = self.data.sensor("imu_gyro").data[1] + self.np_random.uniform(-0.05, 0.05)  # (wx, wy, wz) -> gyros give angular velocity in each axis
        
        # read gt wheel vel with noise (not available on robot - differentiate encoder readings then filter instead)
        wl = self.data.sensor("wheel_left_vel").data[0] + self.np_random.uniform(-0.05, 0.05)  # (angular vel) [0] cos length 1 vector
        wr = self.data.sensor("wheel_right_vel").data[0] + self.np_random.uniform(-0.05, 0.05)

        return np.array([pitch, pitch_rate, wl, wr], dtype=np.float32) # return core obs (without histories)


    def step(self, action):

        # get action to use out of buf first - (front most recent)
        delayed_action = self.action_buf[-self.num_actions:] # last action in buf

        # store latest action in the buffer - to be used N steps later
        self.action_buf = np.roll(self.action_buf, self.num_actions)
        self.action_buf[0:self.num_actions] = action

        
        # torque = policy output clip to [-1, 1] - ACTUATOR DOES SCALING NOW
        self.data.ctrl[:] = np.clip(delayed_action, -1.0, 1.0) # pass raw action to actuator model 

        # advance physics with action input and construct new obs
        for i in range(self.physics_substeps):
            mujoco.mj_step(self.model, self.data)

        # update action history here - front is most recent
        self.action_hist = np.roll(self.action_hist, self.num_actions)  # queue of prev actions
        self.action_hist[0:self.num_actions] = action ## IMPORTANT - MUST be action not delayed_action, policy only knows action it commanded as output

        self.step_count += 1
        obs = self._get_obs()
        full_obs = np.array([*obs, *self.action_hist, *self.obs_hist], dtype=np.float32)  # full obs to return

        # update obs hist here
        self.obs_hist = np.roll(self.obs_hist, self.core_obs_len)
        self.obs_hist[0:self.core_obs_len] = obs

        # ---- REWARDS ---- #
        # use GT vals from sim - not noisy vals from obs

        gt_pitch = self.get_pitch()

        upright = 1.0 - (gt_pitch/self.fall_angle)**2
        #action_pen = -0.001 * np.sum(np.square(delayed_action)) # penalise large torques

        action_rate_pen = 0.0
        if self.prev_action is not None:
            action_rate_pen = -0.002 * np.sum(np.square(delayed_action - self.prev_action))
        
        self.prev_action = delayed_action.copy()

        reward = upright + action_rate_pen #action_pen

        terminated = bool(abs(gt_pitch) > self.fall_angle)
        truncated = bool(self.step_count >= self.max_steps)

        return full_obs, float(reward), terminated, truncated, {}