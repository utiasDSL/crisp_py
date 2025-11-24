"""Try to follow a "figure eight" target on the yz plane."""

# %%
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

from crisp_py.robot import Robot

# left_arm = Robot(namespace="left")
left_arm = Robot()
left_arm.wait_until_ready()

# %%
# print("Going to home position...")
# left_arm.home()
# homing_pose = left_arm.end_effector_pose.copy()


# %%
# Paremeters for the circle
radius = 0.2  # [m]
center = np.array([0.4, 0.0, 0.4])
ctrl_freq = 50.0
sin_freq_y = 0.25  # rot / s
sin_freq_z = 0.125  # rot / s
max_time = 8.0


# %%
# The set_target will directly publish the pose to /target_pose
def drive_eight():
    ee_poses = []
    target_poses = []
    ts = []

    print("Starting to draw a circle...")
    t = 0.0
    target_pose = left_arm.end_effector_pose.copy()
    rate = left_arm.node.create_rate(ctrl_freq)

    while t < max_time:
        x = center[0]
        y = radius * np.sin(2 * np.pi * sin_freq_y * t) + center[1]
        z = radius * np.sin(2 * np.pi * sin_freq_z * t) + center[2]
        target_pose.position = np.array([x, y, z])

        left_arm.set_target(pose=target_pose)

        rate.sleep()

        ee_poses.append(left_arm.end_effector_pose.copy())
        target_poses.append(left_arm.target_pose.copy())
        ts.append(t)

        t += 1.0 / ctrl_freq

    while t < max_time + 1.0:
        # Just wait a bit for the end effector to settle

        rate.sleep()

        ee_poses.append(left_arm.end_effector_pose.copy())
        target_poses.append(left_arm.target_pose.copy())
        ts.append(t)

        t += 1.0 / ctrl_freq
    return ts, target_poses, ee_poses


# %% === 1. OSC ===
left_arm.home()
left_arm.cartesian_controller_parameters_client.load_param_config(
    file_path="config/control/default_operational_space_controller.yaml"
)
left_arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")
left_arm.move_to(position=center, speed=0.05)
ts, target_poses_osc, ee_poses_osc = drive_eight()

# %% === 2. Default CI ===
left_arm.home()
left_arm.cartesian_controller_parameters_client.load_param_config(
    file_path="config/control/default_cartesian_impedance.yaml"
)
left_arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")
left_arm.move_to(position=center, speed=0.05)
ts, target_poses_ci, ee_poses_ci = drive_eight()

# %%
x_t = [target_pose_sample.position[0] for target_pose_sample in target_poses_osc]
y_t = [target_pose_sample.position[1] for target_pose_sample in target_poses_osc]
z_t = [target_pose_sample.position[2] for target_pose_sample in target_poses_osc]

# %%
# === Normal params ===
x_ee_osc = [ee_pose.position[0] for ee_pose in ee_poses_osc]
y_ee_osc = [ee_pose.position[1] for ee_pose in ee_poses_osc]
z_ee_osc = [ee_pose.position[2] for ee_pose in ee_poses_osc]
x_ee_ci = [ee_pose.position[0] for ee_pose in ee_poses_ci]
y_ee_ci = [ee_pose.position[1] for ee_pose in ee_poses_ci]
z_ee_ci = [ee_pose.position[2] for ee_pose in ee_poses_ci]


# fig, ax = plt.subplots(1, 3, figsize=(10, 5))
fig = plt.figure(figsize=(12, 4))  # width x height
gs = gridspec.GridSpec(1, 5, figure=fig)
ax0 = fig.add_subplot(gs[0, 0])
ax1 = fig.add_subplot(gs[0, 1:3])
ax2 = fig.add_subplot(gs[0, 3:6])

ax0.plot(x_ee_osc, z_ee_osc, label="OSC")
ax0.plot(x_ee_ci, z_ee_ci, label="CI")
ax0.plot(x_t, z_t, label="target", linestyle="--")
ax0.set_xlim([0.3, 0.5])
ax0.set_xticks(np.arange(0.3, 0.505, 0.05))
ax0.set_xlabel("$x$")
ax0.set_ylabel("$z$")
ax1.plot(y_ee_osc, z_ee_osc, label="OSC")
ax1.plot(y_ee_ci, z_ee_ci, label="CI")
ax1.plot(y_t, z_t, label="target", linestyle="--")
ax1.set_xlabel("$y$")
ax1.set_ylabel("$z$")
ax2.plot(ts, z_ee_osc, label="OSC")
ax2.plot(ts, z_ee_ci, label="CI")
ax2.plot(ts, z_t, label="target", linestyle="--")
ax2.set_xlabel("$t$")
ax2.legend()

ax0.grid()
ax1.grid()
ax2.grid()

fig.tight_layout()

plt.show()

# %%

print("Going back home.")
left_arm.home()

# %%
left_arm.shutdown()
