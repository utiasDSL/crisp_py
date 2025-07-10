"""Simple example to control the gripper."""

# %%
import matplotlib.pyplot as plt
import numpy as np
import time

from crisp_py.gripper.gripper import GripperConfig
from crisp_py.robot import Robot
from crisp_py.robot_config import SO101Config

from crisp_py.gripper import Gripper

robot = Robot(namespace="so_100_arm", robot_config=SO101Config())
robot.wait_until_ready()
gripper = Gripper(namespace="so_100_arm", gripper_config=GripperConfig(min_value=0.0, max_value=1.0, joint_state_topic="gripper_value"))
gripper.wait_until_ready()

# %%

print(robot.end_effector_pose)
print(robot.joint_values)
print(gripper.value)
