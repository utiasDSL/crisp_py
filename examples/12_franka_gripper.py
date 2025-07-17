"""Simple example to control the gripper."""

# %%
import time

from crisp_py.gripper import Gripper, GripperConfig

# %%
config = GripperConfig.from_yaml(path="config/gripper_franka.yaml")
gripper = Gripper(gripper_config=config, namespace="right")
gripper.wait_until_ready()

# %%
gripper.value

# Almost fully open
gripper.open()

time.sleep(3.0)

# Almost fully closed
gripper.close()
