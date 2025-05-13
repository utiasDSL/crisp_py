"""Simple example to control the gripper."""

# %%
import time
from pathlib import Path

import yaml
from crisp_py.gripper.gripper import Gripper, GripperConfig

project_root_path = Path("/home/lsy_franka/repos/crisp_py")

right_config = None, None
with open(project_root_path / "config" / "gripper_right_config.yaml", "r") as file:
    right_config = GripperConfig(**yaml.safe_load(file))

# %%

gripper = Gripper(namespace="follower", gripper_config=right_config)
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

# Almost fully open
gripper.set_target(0.9)

time.sleep(3.0)

# Almost fully closed
gripper.set_target(0.1)
