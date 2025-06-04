"""Simple example to control the gripper."""

# %%
import time
from pathlib import Path

import yaml

from crisp_py.gripper.gripper import Gripper, GripperConfig

project_root_path = Path("/home/quisty/repos/crisp_py")

config = None
with open(project_root_path / "config" / "gripper_franka.yaml", "r") as file:
    config = yaml.safe_load(file)
    config = GripperConfig(min_value=config.get("min_value"), max_value=config.get("max_value"))

# %%

gripper = Gripper(gripper_config=config, namespace="")
gripper.wait_until_ready()

# %%
gripper.value

# Almost fully open
gripper.set_target(1.0)

time.sleep(3.0)

# Almost fully closed
gripper.set_target(0.0)

time.sleep(3.0)
gripper.set_target(1.0)