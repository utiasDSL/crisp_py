"""Simple example to control the gripper.

In this example we use a trigger to control the gripper.
The leader gripper and the follower gripper are controlled by the trigger.
"""
# %%
import time
from crisp_py.gripper.gripper import Gripper, GripperConfig


trigger_config = GripperConfig.from_yaml("config/trigger.yaml")
trigger = Gripper(gripper_config=trigger_config, namespace="left_gripper/gripper", index=1)
trigger.wait_until_ready()

left_config = GripperConfig.from_yaml("config/gripper_with_trigger.yaml")
left_gripper = Gripper(node=trigger.node, gripper_config=left_config, namespace="left_gripper/gripper")
left_gripper.wait_until_ready()


right_config = GripperConfig.from_yaml("config/gripper_right.yaml")
right_gripper = Gripper(gripper_config=right_config, namespace="right_gripper/gripper")
right_gripper.wait_until_ready()

# %%
rate = left_gripper.node.create_rate(50)
start_time = time.time()
max_time = 30.0  # seconds

right_gripper.enable_torque()
left_gripper.enable_torque()

while time.time() - start_time < max_time:
    left_gripper.set_target(max(min(1.0, trigger.value if trigger.value is not None else 0.0), 0.0))
    right_gripper.set_target(max(min(1.0, trigger.value if trigger.value is not None else 0.0), 0.0))
    rate.sleep()
