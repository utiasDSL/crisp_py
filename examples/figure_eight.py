"""Try to follow a "figure eight" target on the yz plane."""

import pinocchio as pin
import numpy as np

# %%
from crisp_py.robot import Robot

robot = Robot(namespace="right")
robot.wait_until_ready()

# %%
print(robot.end_effector_pose)

# %%
print("Going to home position...")
robot.home()

# %%
# Paremeters for the circle
radius = 0.2  # [m]
center = [0.4, 0.0, 0.4]
ctrl_freq = 50.0
sin_freq_y = 0.25  # rot / s
sin_freq_z = 0.125  # rot / s
max_time = 20.0
# %%

robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

# %%
# The move_to function will publish a pose to /target_pose while interpolation linearly
starting_pose = pin.SE3(
    robot.end_effector_pose.rotation,
    np.array(center),
)

robot.move_to(pose=starting_pose, speed=0.15)

# %%
# The set_target will directly publish the pose to /target_pose
ee_poses = []
target_poses = []

print("Starting to draw a circle...")
t = 0.0
target_pose = robot.end_effector_pose.copy()
rate = robot.node.create_rate(ctrl_freq)

while t < max_time:
    x = center[0]
    y = radius * np.sin(2 * np.pi * sin_freq_y * t) + center[1]
    z = radius * np.sin(2 * np.pi * sin_freq_z * t) + center[2]
    target_pose.translation = np.array([x, y, z])

    robot.set_target(pose=target_pose)

    rate.sleep()

    ee_poses.append(robot.end_effector_pose.copy())
    target_poses.append(robot._target_pose.copy())

    t += 1.0 / ctrl_freq


print("Done drawing a circle!")

# %%
print(len(ee_poses))
print(len(target_poses))

# %%
y_ee = [ee_pose.translation[1] for ee_pose in ee_poses]
y_t = [target_pose_sample.translation[1] for target_pose_sample in target_poses]
z_ee = [ee_pose.translation[2] for ee_pose in ee_poses]
z_t = [target_pose_sample.translation[2] for target_pose_sample in target_poses]
# %%
import matplotlib.pyplot as plt

plt.plot(y_ee, z_ee, label="current")
plt.plot(y_t, z_t, label="target")
plt.xlabel("$y$")
plt.ylabel("$z$")
plt.legend()
plt.show()

# %%
print("Going back home.")
robot.home()

# %%
robot.shutdown()

# %%
