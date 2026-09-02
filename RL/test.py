import mujoco
import mujoco_warp as mjw
import torch
import warp as wp

print("=" * 50)
print("1. PYTORCH CUDA CHECK")
print("=" * 50)
print(f"CUDA Available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"Device Name:    {torch.cuda.get_device_name(0)}")
    print(f"Device Count:   {torch.cuda.device_count()}")
    # Test tensor allocation directly in VRAM
    x = torch.ones((3, 3), device="cuda:0")
    print(f"VRAM Tensor:    {x.device} (OK)")
else:
    print("Warning: PyTorch cannot access the GPU.")

print("\n" + "=" * 50)
print("2. NVIDIA WARP & MUJOCO-WARP CHECK")
print("=" * 50)
wp.init()
warp_device = wp.get_device()
print(f"Warp Device:    {warp_device}")

# Simple free-falling sphere XML
xml = """
<mujoco>
  <worldbody>
    <geom type="plane" size="1 1 0.1"/>
    <body pos="0 0 1">
      <freejoint/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
</mujoco>
"""

# Compile host MuJoCo model
mj_model = mujoco.MjModel.from_xml_string(xml)

# Copy model to GPU and allocate 16 batched environments
model = mjw.put_model(mj_model)
data = mjw.make_data(mj_model, nworld=16)

# Step simulation on GPU
mjw.step(model, data)

# Verify position array lives on CUDA device
qpos_device = data.qpos.device
print(f"MJWarp step:    Successfully stepped 16 worlds on {qpos_device}!")
print("=" * 50)