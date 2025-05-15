"""Rotate the target around the z-axis, this is an interesting edge case that can easily lead to failure... In which direction should the gripper turn?."""

# %%
import numpy as np
import pinocchio as pin

from crisp_py.robot import Robot

robot = Robot()
robot.wait_until_ready()

# %%
robot.home()
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
homing_pose = robot.end_effector_pose.copy()

# %%

rotate_90 = pin.exp3(np.array([0.0, 0.0, +np.pi / 2.0]))
rotate_m180 = pin.exp3(np.array([0.0, 0.0, -np.pi]))

# %%
first_pose = homing_pose.copy()
first_pose.rotation = pin.exp3(pin.log3(first_pose.rotation @ rotate_90))

second_pose = first_pose.copy()
second_pose.rotation = pin.exp3(pin.log3(second_pose.rotation @ rotate_m180))

# The set_target will directly publish the pose to /target_pose

ctrl_freq = 20.0

print("Starting to draw a circle...")
target_pose = robot.end_effector_pose.copy()
rate = robot.node.create_rate(ctrl_freq)


loop = 0
max_loops = 10

while loop < max_loops:
    t = 0.0
    while t < 10.0:
        if t < 5.0:
            robot.set_target(pose=first_pose)
        else:
            robot.set_target(pose=second_pose)

        rate.sleep()
        t += 1.0 / ctrl_freq
