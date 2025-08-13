"""Try to follow a "figure eight" target on the yz plane."""

# %%
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from crisp_py.robot import Robot

left_arm = Robot(namespace="left")
right_arm = Robot(namespace="right")
left_arm.wait_until_ready()
right_arm.wait_until_ready()

# %%
left_arm.home()
right_arm.home()


# %%
left_arm.cartesian_controller_parameters_client.load_param_config(
    file_path="config/control/gravity_compensation.yaml"
)
right_arm.cartesian_controller_parameters_client.load_param_config(
    file_path="config/control/default_cartesian_impedance.yaml"
)

# %%
left_arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")
right_arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")


def drive_trajectory(target_pose, max_time=10.0):
    ctrl_freq = 100.0
    ee_poses = []
    target_poses = []
    ts = []

    rate = right_arm.node.create_rate(ctrl_freq)

    t = 0.0
    while t < max_time:
        if t > 1.0:
            right_arm.set_target(pose=target_pose)

        rate.sleep()

        ee_poses.append(right_arm.end_effector_pose.copy())
        target_poses.append(right_arm.target_pose.copy())
        ts.append(t)

        t += 1.0 / ctrl_freq

    return ts, ee_poses, target_poses


# %%
def get_error(target_poses, ee_poses):
    """Get the error between the end effector and the target pose."""
    # We extract some values from the trajetory
    x_t = np.array([target_pose_sample.position[0] for target_pose_sample in target_poses])
    y_t = np.array([target_pose_sample.position[1] for target_pose_sample in target_poses])
    z_t = np.array([target_pose_sample.position[2] for target_pose_sample in target_poses])
    R_t = np.array([target_pose_sample.rotation for target_pose_sample in target_poses])
    x_ee = np.array([ee_pose_sample.position[0] for ee_pose_sample in ee_poses])
    y_ee = np.array([ee_pose_sample.position[1] for ee_pose_sample in ee_poses])
    z_ee = np.array([ee_pose_sample.position[2] for ee_pose_sample in ee_poses])
    R_ee = np.array([ee_pose_sample.rotation for ee_pose_sample in ee_poses])

    # And compute the error i.e. differences between the end effector and the target
    dx = x_ee - x_t
    dy = y_ee - y_t
    dz = z_ee - z_t
    d_rot = np.array([Rotation.from_matrix(R_diff).as_rotvec() for R_diff in R_ee @ R_t.transpose(0, 2, 1)])
    drx = d_rot[:, 0]
    dry = d_rot[:, 1]
    drz = d_rot[:, 2]
    return dx, dy, dz, drx, dry, drz


# %%
print("Move the left arm to a desired pose, press enter after choosing your target pose.")
input()
target_pose = left_arm.end_effector_pose.copy()


print("Now, after pressing enter again, the right arm will drive to that pose. Careful!")
input()

# %%

right_arm.cartesian_controller_parameters_client.load_param_config(
    file_path="config/control/default_cartesian_impedance.yaml"
)
right_arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")

ts, ee_poses, target_poses = drive_trajectory(target_pose=target_pose, max_time=5.0)
dx, dy, dz, drx, dry, drz = get_error(target_poses, ee_poses)
error_pos = np.sqrt(dx**2 + dy**2 + dz**2)
error_rot = np.sqrt(drx**2 + dry**2 + drz**2)

right_arm.home()

# %%

right_arm.cartesian_controller_parameters_client.load_param_config(
    file_path="config/control/clipped_cartesian_impedance.yaml"
)
right_arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")

ts, ee_clip, target_poses = drive_trajectory(target_pose=target_pose, max_time=5.0)
dx_clip, dy_clip, dz_clip, drx_clip, dry_clip, drz_clip = get_error(target_poses, ee_clip)
error_pos_clip = np.sqrt(dx_clip**2 + dy_clip**2 + dz_clip**2)
error_rot_clip = np.sqrt(drx_clip**2 + dry_clip**2 + drz_clip**2)
right_arm.home(blocking=False)

# %%

mpl.rcParams.update(
    {
        "font.family": "serif",
        "font.size": 12,
        "axes.labelsize": 14,
        "axes.titlesize": 14,
        "legend.fontsize": 12,
        "xtick.labelsize": 12,
        "ytick.labelsize": 12,
    }
)

fig, ax = plt.subplots(2, 1, figsize=(8, 10), sharex=True)

ax[0].plot(ts, error_pos, label="Standard", linewidth=1.5)
ax[0].plot(ts, error_pos_clip, label="Clipped", linewidth=1.5, linestyle="--")
ax[0].set_ylabel(r"$e_{\mathrm{pos}}$")
ax[0].legend(loc="upper right")
ax[0].grid(True)

steady_idx = -1
ax[0].annotate(
    f"{error_pos_clip[steady_idx] * 1000.0:2f} [mm]",
    xy=(ts[steady_idx], error_pos[steady_idx]),
    xytext=(ts[steady_idx] - 1, error_pos[steady_idx] + 0.05),
    fontsize=12,
)
ax[0].annotate(
    f"{error_pos[steady_idx] * 1000.0:2f} [mm]",
    xy=(ts[steady_idx], error_pos[steady_idx]),
    xytext=(ts[steady_idx] - 1, error_pos[steady_idx] + 0.06),
    fontsize=12,
)

ax[1].plot(ts, error_rot, label="Standard", linewidth=1.5)
ax[1].plot(ts, error_rot_clip, label="Clipped", linewidth=1.5, linestyle="--")
ax[1].set_ylabel(r"$e_{\mathrm{rot}}$")
ax[1].set_xlabel("Time [s]")
ax[1].grid(True)

ax[1].annotate(
    f"{error_rot_clip[steady_idx]:2f} [rad]",
    xy=(ts[steady_idx], error_rot[steady_idx]),
    xytext=(ts[steady_idx] - 1, error_rot[steady_idx] + 0.05),
    fontsize=12,
)

plt.tight_layout()
plt.show()

# fig, ax = plt.subplots(2, 1, figsize=(8, 10))
# ax[0].plot(ts, error_pos, label="standard")
# ax[0].plot(ts, error_pos_clip, label="clipped")
# ax[0].set_ylabel("$e_{\\text{pos}}$")
# ax[0].legend()
# ax[0].grid()
#
# ax[1].plot(ts, error_rot, label="standard")
# ax[1].plot(ts, error_rot_clip, label="clipped")
# ax[1].set_ylabel("$e_{\\text{rot}}$")
# ax[1].grid()
#
# plt.show()

# %%

print("Going back home.")
right_arm.home()
left_arm.home()

# %%
right_arm.shutdown()
left_arm.shutdown()
