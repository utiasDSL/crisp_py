"""Simple example to control the gripper."""

# %%
from crisp_py.gripper.gripper import Gripper

# robot = Robot(namespace="left")
robot = Robot()
robot.wait_until_ready()

# %%
print(robot.end_effector_pose)
print(robot.joint_values)

# %%
print("Going to home position...")
robot.home()
homing_pose = robot.end_effector_pose.copy()
