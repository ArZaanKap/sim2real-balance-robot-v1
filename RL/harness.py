import time
import numpy as np
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("models/model0.xml")
data = mujoco.MjData(model) # whats in data?

# add small angular vel around y axis
data.qvel[4] = 0.05 # qvel = [vx, vy, vz, wx, wy, wz]

step = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        mujoco.mj_step(model, data) # advance physics
        step += 1

        if step % 50 == 0:  # 10hz (500/50)
            quat = data.sensor("imu_quat").data
            gyro = data.sensor("imu_gyro").data
            wl = data.sensor("wheel_left_vel").data[0] # (lin vel, angular vel)?
            wr = data.sensor("wheel_right_vel").data[0]

            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, quat) # quat -> 3x3 rotation matrix
            R = R.reshape(3,3)
            pitch = np.arctan2(R[0,2], R[2,2]) # explain math?

            print(f"t={data.time:5.2f} pitch={np.degrees(pitch):+6.1f}deg  "f"pitch_rate={gyro[1]:+6.2f} wheelL={wl:+6.2f}  wheelR={wr:+6.2f}")

        viewer.sync() # push new state to viewing window

        # default timestep is 0.002s (500Hz)
        dt = model.opt.timestep - (time.time() - step_start)  # sleep till next physics time step        
        if dt > 0.0:
            time.sleep(dt)