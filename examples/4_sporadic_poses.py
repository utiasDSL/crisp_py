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
    ("use_operational_space", False),
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
x_t = [target_pose_sample.translation[0] for target_pose_sample in target_poses]
y_t = [target_pose_sample.translation[1] for target_pose_sample in target_poses]
z_t = [target_pose_sample.translation[2] for target_pose_sample in target_poses]
rx_t = [pin.log3(target_pose_sample.rotation)[0] for target_pose_sample in target_poses]
ry_t = [pin.log3(target_pose_sample.rotation)[1] for target_pose_sample in target_poses]
rz_t = [pin.log3(target_pose_sample.rotation)[2] for target_pose_sample in target_poses]

# %%
x_ee = np.array([ee_pose_sample.translation[0] for ee_pose_sample in ee_poses])
y_ee = np.array([ee_pose_sample.translation[1] for ee_pose_sample in ee_poses])
z_ee = np.array([ee_pose_sample.translation[2] for ee_pose_sample in ee_poses])
rx_ee = np.array([pin.log3(ee_pose_sample.rotation)[0] for ee_pose_sample in ee_poses])
ry_ee = np.array([pin.log3(ee_pose_sample.rotation)[1] for ee_pose_sample in ee_poses])
rz_ee = np.array([pin.log3(ee_pose_sample.rotation)[2] for ee_pose_sample in ee_poses])

# %%
import matplotlib.pyplot as plt
# %%

se_x = (x_ee - x_t) ** 2
se_y = (y_ee - y_t) ** 2
se_z = (z_ee - z_t) ** 2
se_rx = (rx_ee - rx_t) ** 2
se_ry = (ry_ee - ry_t) ** 2
se_rz = (rz_ee - rz_t) ** 2

fig, ax = plt.subplots(2, 1, figsize=(8, 10))
ax[0].plot(ts, se_x, label="SE x")
ax[0].plot(ts, se_y, label="SE y")
ax[0].plot(ts, se_z, label="SE y")
ax[1].plot(ts, se_rx, label="SE rx")
ax[1].plot(ts, se_ry, label="SE ry")
ax[1].plot(ts, se_rz, label="SE rz")

plt.legend()

plt.show()

# %%

print("Going back home.")
robot.home()

# %%
robot.shutdown()

# %%
