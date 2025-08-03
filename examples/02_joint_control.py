"""Example controlling the joints."""
import time

import numpy as np

from crisp_py.robot import Robot
from crisp_py.robot_config import FrankaConfig

robot_config = FrankaConfig(publish_frequency=100.0, target_joint_topic="target_joint")
robot = Robot(namespace="right")
robot.wait_until_ready()

# %%
robot.controller_switcher_client.switch_controller("joint_impedance_controller")

# %%
q = robot.joint_values
q[0] += 0.2
robot.set_target_joint(q)

time.sleep(1.0)

robot.shutdown()