"""Simple example to control the gripper."""

# %%
from pathlib import Path

import yaml

from crisp_py.gripper.gripper import Gripper, GripperConfig

project_root_path = Path("/home/lsy_franka/repos/crisp_py")

left_config, right_config = None, None
with open(project_root_path / "config" / "gripper_left_config.yaml", "r") as file:
    config = yaml.safe_load(file)
    left_config = GripperConfig(
        min_value=config.get("min_value"), max_value=config.get("max_value")
    )

with open(project_root_path / "config" / "gripper_right_config.yaml", "r") as file:
    config = yaml.safe_load(file)
    right_config = GripperConfig(
        min_value=config.get("min_value"), max_value=config.get("max_value")
    )

# %%

leader_gripper = Gripper(namespace="leader", gripper_config=left_config)
follower_gripper = Gripper(namespace="follower", gripper_config=right_config)
leader_gripper.wait_until_ready()
follower_gripper.wait_until_ready()

# %%
print(follower_gripper.value)
print(leader_gripper.value)


# %%
follower_gripper.min_value
follower_gripper.max_value
follower_gripper._unnormalize(0.5)

# %%
follower_gripper.set_target(0.5)

# %%

freq = 50.0
rate = follower_gripper.node.create_rate(freq)
dt = 1.0 / freq

t = 0.0
while True:
    follower_gripper.set_target(leader_gripper.value)
    rate.sleep()
    t += dt
