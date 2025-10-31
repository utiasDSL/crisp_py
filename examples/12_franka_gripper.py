"""Simple example to control the gripper."""

# %%
import time

from crisp_py.gripper import Gripper, GripperConfig

# %%
config = GripperConfig.from_yaml(path="config/gripper_franka.yaml")


config.max_delta = 10.0
print(config)
gripper = Gripper(gripper_config=config)
print(gripper._joint_state_sub.topic_name)
print(gripper.wait_until_ready())

# %%
gripper.value

# Almost fully open
gripper.open()

time.sleep(3.0)

# Almost fully closed
gripper.close()

time.sleep(3.0)
input("Waiting for input")

gripper.open()

time.sleep(3.0)

gripper.shutdown()
