"""Simple example to control the gripper."""

# %%
import time
from pathlib import Path

import yaml

from crisp_py.gripper.gripper import Gripper, GripperConfig

project_root_path = Path("/home/lsy_franka/repos/crisp_py")

right_config = None
with open(project_root_path / "config" / "gripper_right_config.yaml", "r") as file:
    config = yaml.safe_load(file)
    right_config = GripperConfig(
        min_value=config.get("min_value"), max_value=config.get("max_value")
    )

# %%

gripper = Gripper(gripper_config=right_config, namespace="follower")
gripper.wait_until_ready()

# %%
freq = 1.0
rate = gripper.node.create_rate(freq)
t = 0.0
while t < 10.0:
    print(gripper.value)
    print(gripper.torque)
    rate.sleep()
    t += 1.0 / freq

# %%
gripper.value

# Almost fully open
gripper.set_target(0.9)

time.sleep(3.0)

# Almost fully closed
gripper.set_target(0.1)

# %%
try:
    gripper.reboot()
except RuntimeError as e:
    print(e)

# %%
try:
    gripper.enable_torque()
except RuntimeError as e:
    print(e)

# %%
try:
    gripper.disable_torque()
except RuntimeError as e:
    print(e)
