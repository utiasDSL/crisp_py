"""Try to follow a "figure eight" target on the yz plane."""

import pinocchio as pin
import numpy as np
from rcl_interfaces.srv import GetParameters, ListParameters

# %%
from crisp_py.robot import Robot

robot = Robot(namespace="right")
robot.wait_until_ready()

# %%
print(robot.end_effector_pose)

# %%
print("Going to home position...")
robot.home()
homing_pose = robot.end_effector_pose.copy()

# %%
# Paremeters for the circle
radius = 0.2  # [m]
center = [0.4, 0.0, 0.4]
ctrl_freq = 50.0
sin_freq_y = 0.25  # rot / s
sin_freq_z = 0.125  # rot / s
max_time = 8.0
# %%

robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

# %%
# The move_to function will publish a pose to /target_pose while interpolation linearly
starting_pose = pin.SE3(
    homing_pose.rotation,
    np.array(center),
)

robot.move_to(pose=starting_pose, speed=0.15)

# %%
# The set_target will directly publish the pose to /target_pose
ee_poses = []
target_poses = []
ts = []

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
    ts.append(t)

    t += 1.0 / ctrl_freq

while t < max_time + 1.0:
    # Just wait a bit for the end effector to settle

    rate.sleep()

    ee_poses.append(robot.end_effector_pose.copy())
    target_poses.append(robot._target_pose.copy())
    ts.append(t)

    t += 1.0 / ctrl_freq


print("Done drawing a circle!")

# %%
print(len(ee_poses))
print(len(target_poses))

# %%
y_t = [target_pose_sample.translation[1] for target_pose_sample in target_poses]
z_t = [target_pose_sample.translation[2] for target_pose_sample in target_poses]

# %%
# === Normal params ===
y_ee = [ee_pose.translation[1] for ee_pose in ee_poses]
z_ee = [ee_pose.translation[2] for ee_pose in ee_poses]

# %%
# === Stiffer params ===
y_stiffer = [ee_pose.translation[1] for ee_pose in ee_poses]
z_stiffer = [ee_pose.translation[2] for ee_pose in ee_poses]

# %%
# === No coriolis comp ===
y_nocor = [ee_pose.translation[1] for ee_pose in ee_poses]
z_nocor = [ee_pose.translation[2] for ee_pose in ee_poses]

# %%
import matplotlib.pyplot as plt

plt.plot(y_ee, z_ee, label="current")
plt.plot(y_nocor, z_nocor, label="no corriolis")
plt.plot(y_stiffer, z_stiffer, label="stiffer")
plt.plot(y_t, z_t, label="target", linestyle="--")
plt.xlabel("$y$")
plt.ylabel("$z$")
plt.legend()
plt.show()

# %%
plt.plot(ts, z_ee, label="current")
plt.plot(ts, z_nocor, label="no corriolis")
plt.plot(ts, z_stiffer, label="stiffer")
plt.plot(ts, z_t, label="target", linestyle="--")
plt.xlabel("$t$")
plt.ylabel("$z$")
plt.legend()
plt.show()

# %%
fig, ax = plt.subplots(1, 2, figsize=(10, 5))
ax[0].plot(y_ee, z_ee, label="current")
ax[0].plot(y_nocor, z_nocor, label="no corriolis")
ax[0].plot(y_stiffer, z_stiffer, label="stiffer")
ax[0].plot(y_t, z_t, label="target", linestyle="--")
ax[0].set_xlabel("$y$")
ax[0].set_ylabel("$z$")
# ax[0].legend()
ax[1].plot(ts, z_ee, label="current")
ax[1].plot(ts, z_nocor, label="no corriolis")
ax[1].plot(ts, z_stiffer, label="stiffer")
ax[1].plot(ts, z_t, label="target", linestyle="--")
ax[1].set_xlabel("$t$")
ax[1].legend()

for a in ax:
    a.grid()

fig.tight_layout()

plt.show()

# %%

print("Going back home.")
robot.home()

# %%
robot.shutdown()

# %%
