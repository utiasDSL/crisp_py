"""Try to follow a "figure eight" target on the yz plane."""

# %%
import numpy as np
import pinocchio as pin

from crisp_py.robot import Robot

# robot = Robot(namespace="right")
robot = Robot()
robot.wait_until_ready()

# %%
robot.home()
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
homing_pose = robot.end_effector_pose.copy()

# %%
first_pose = homing_pose.copy()
first_pose.translation[0] += 0.2
first_pose.translation[1] += 0.2
first_pose.translation[2] += 0.1
# robot.set_target(pose=first_pose)

# %%
second_pose = first_pose.copy()
second_pose.translation[1] += 0.2
second_pose.translation[2] -= 0.2
transform_rotation = pin.exp3(np.array([0.0, 0.0, +np.pi / 2.0]))
second_pose.rotation = pin.exp3(pin.log3(second_pose.rotation @ transform_rotation))
# robot.set_target(pose=second_pose)

# %%
third_pose = second_pose.copy()
third_pose.translation[1] -= 0.6
transform_rotation = pin.exp3(np.array([0.0, -np.pi / 2, 0.0]))
third_pose.rotation = pin.exp3(pin.log3(third_pose.rotation @ transform_rotation))
# robot.set_target(pose=third_pose)

# robot.set_target(pose=second_pose)
# print(second_pose)
# print(second_pose.rotation)
# print(pin.log3(second_pose.rotation))
# print(second_pose)

# %%
params = [
    ("task.k_pos_x", 400.0),
    ("task.k_pos_y", 400.0),
    ("task.k_pos_z", 400.0),
    ("task.k_rot_x", 100.0),
    ("task.k_rot_y", 100.0),
    ("task.k_rot_z", 100.0),
    ("nullspace.stiffness", 0.0),
    ("nullspace.projector_type", "kinematic"),
    ("use_operational_space", True),
    ("use_local_jacobian", True),
]
robot.cartesian_controller_parameters_client.set_parameters(params)

# %%

# The set_target will directly publish the pose to /target_pose

ctrl_freq = 20.0
ee_poses = []
target_poses = []
ts = []

print("Starting to draw a circle...")
target_pose = robot.end_effector_pose.copy()
rate = robot.node.create_rate(ctrl_freq)

max_time = 30.0

t = 0.0
while t < max_time:
    if t < 10.0:
        robot.set_target(pose=first_pose)
    elif t < 20.0:
        robot.set_target(pose=second_pose)
    else:
        robot.set_target(pose=third_pose)

    rate.sleep()

    ee_poses.append(robot.end_effector_pose.copy())
    target_poses.append(robot.target_pose.copy())
    ts.append(t)

    t += 1.0 / ctrl_freq

while t < max_time + 1.0:
    # Just wait a bit for the end effector to settle

    rate.sleep()

    ee_poses.append(robot.end_effector_pose.copy())
    target_poses.append(robot._target_pose.copy())
    ts.append(t)

    t += 1.0 / ctrl_freq


print("Done driving to poses.")
robot.home()
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

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
import matplotlib.pyplot as plt
# %%

plt.plot(y_ee, z_ee, label="current")
plt.plot(y_t, z_t, label="target", linestyle="--")
plt.xlabel("$y$")
plt.ylabel("$z$")
plt.legend()
plt.show()

# %%
plt.plot(ts, z_ee, label="current")
plt.plot(ts, z_t, label="target", linestyle="--")
plt.xlabel("$t$")
plt.ylabel("$z$")
plt.legend()
plt.show()

# %%
fig, ax = plt.subplots(1, 2, figsize=(10, 5))
ax[0].plot(y_ee, z_ee, label="current")
ax[0].plot(y_t, z_t, label="target", linestyle="--")
ax[0].set_xlabel("$y$")
ax[0].set_ylabel("$z$")
# ax[0].legend()
ax[1].plot(ts, z_ee, label="current")
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
