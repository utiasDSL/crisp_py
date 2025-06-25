"""Try to follow a "figure eight" target on the yz plane."""

# %%
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin

from crisp_py.robot import Robot

robot = Robot(namespace="right")
# robot = Robot()
robot.wait_until_ready()

# %%
robot.home()
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
homing_pose = robot.end_effector_pose.copy()

# %%

params_names = robot.cartesian_controller_parameters_client.list_parameters()
i = 0
for param_name in params_names:
    print(param_name, end="\n" if i % 2 == 1 else ", ")
    i += 1


# %%
first_pose = homing_pose.copy()
first_pose.position[0] += 0.1
first_pose.position[1] += 0.3
first_pose.position[2] -= 0.1
transform_rotation = pin.exp3(np.array([0.0, 0.0, +np.pi / 2.0]))
first_pose.rotation = pin.exp3(pin.log3(first_pose.rotation @ transform_rotation))

# %%
second_pose = first_pose.copy()
second_pose.position[1] -= 0.2
transform_rotation = pin.exp3(np.array([0.0, -np.pi / 2, 0.0]))
second_pose.rotation = pin.exp3(pin.log3(second_pose.rotation @ transform_rotation))


# %%
robot.set_target(pose=first_pose)
# %%


def drive_trajectory():
    ctrl_freq = 20.0
    ee_poses = []
    target_poses = []
    ts = []

    rate = robot.node.create_rate(ctrl_freq)

    max_time = 10.0

    t = 0.0
    while t < max_time:
        if t < 1.0:
            robot.set_target(pose=first_pose)
        else:
            robot.set_target(pose=second_pose)

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

    robot.set_target(pose=first_pose)

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
    d_rot = np.array([pin.log3(R_diff) for R_diff in R_ee @ R_t.transpose(0, 2, 1)])
    drx = d_rot[:, 0]
    dry = d_rot[:, 1]
    drz = d_rot[:, 2]
    return dx, dy, dz, drx, dry, drz


# %%
# Drive the normal trajectory
robot.cartesian_controller_parameters_client.set_parameters(
    [
        ("task.k_pos_x", 500.0),
        ("task.k_pos_y", 500.0),
        ("task.k_pos_z", 500.0),
        ("task.k_rot_x", 20.0),
        ("task.k_rot_y", 20.0),
        ("task.k_rot_z", 20.0),
        ("nullspace.stiffness", 1.0),
        ("nullspace.projector_type", "kinematic"),
        ("use_operational_space", False),
        ("use_local_jacobian", True),
    ]
)
ts, ee_poses, target_poses = drive_trajectory()
dx, dy, dz, drx, dry, drz = get_error(target_poses, ee_poses)

# %%
robot.cartesian_controller_parameters_client.set_parameters(
    [
        ("task.k_pos_x", 1500.0),
        ("task.k_pos_y", 1500.0),
        ("task.k_pos_z", 1500.0),
        ("task.k_rot_x", 100.0),
        ("task.k_rot_y", 100.0),
        ("task.k_rot_z", 100.0),
        # ===
        ("task.d_pos_x", 100.0),
        ("task.d_pos_y", 100.0),
        ("task.d_pos_z", 100.0),
        ("task.d_rot_x", 1.0),
        ("task.d_rot_y", 1.0),
        ("task.d_rot_z", 1.0),
        # ===
        ("task.error_clip.x", 0.01),
        ("task.error_clip.y", 0.01),
        ("task.error_clip.z", 0.01),
        ("task.error_clip.rx", 0.05),
        ("task.error_clip.ry", 0.05),
        ("task.error_clip.rz", 0.05),
        # ===
        ("nullspace.stiffness", 0.0),
        ("nullspace.projector_type", "kinematic"),
        ("use_operational_space", False),
        ("use_local_jacobian", True),
    ]
)
ts, ee_poses, target_poses = drive_trajectory()
dx_clip, dy_clip, dz_clip, drx_clip, dry_clip, drz_clip = get_error(target_poses, ee_poses)

# %%
# Drive the clipper trajectory
robot.cartesian_controller_parameters_client.set_parameters(
    [
        ("task.k_pos_x", 200.0),
        ("task.k_pos_y", 200.0),
        ("task.k_pos_z", 200.0),
        ("task.k_rot_x", 20.0),
        ("task.k_rot_y", 20.0),
        ("task.k_rot_z", 20.0),
        # ===
        ("task.d_pos_x", 0.0),
        ("task.d_pos_y", 0.0),
        ("task.d_pos_z", 0.0),
        ("task.d_rot_x", 0.0),
        ("task.d_rot_y", 0.0),
        ("task.d_rot_z", 0.0),
        # ===
        ("task.error_clip.x", 20.0),
        ("task.error_clip.y", 20.0),
        ("task.error_clip.z", 20.0),
        ("task.error_clip.rx", 15.0),
        ("task.error_clip.ry", 15.0),
        ("task.error_clip.rz", 15.0),
        # ===
        ("nullspace.stiffness", 5.0),
        ("nullspace.projector_type", "kinematic"),
        ("use_operational_space", False),
        ("use_local_jacobian", True),
    ]
)
ts, ee_poses, target_poses = drive_trajectory()
dx_clip, dy_clip, dz_clip, drx_clip, dry_clip, drz_clip = get_error(target_poses, ee_poses)

# %%


fig, ax = plt.subplots(6, 1, figsize=(8, 10))
ax[0].plot(ts, dx, label="standard")
ax[0].plot(ts, dx_clip, label="clipped")
# Add a stripped line going through the zero
ax[0].plot(ts, np.zeros_like(dx), linestyle="--", color="gray", linewidth=0.75)
ax[0].set_ylabel("$\Delta x$")
ax[0].legend()

ax[1].plot(ts, dy, label="standard")
ax[1].plot(ts, dy_clip, label="clipped")
ax[1].plot(ts, np.zeros_like(dy), linestyle="--", color="gray", linewidth=0.75)
ax[1].set_ylabel("$\Delta y$")

ax[2].plot(ts, dz, label="standard")
ax[2].plot(ts, dz_clip, label="clipped")
ax[2].plot(ts, np.zeros_like(dz), linestyle="--", color="gray", linewidth=0.75)
ax[2].set_ylabel("$\Delta z$")

ax[3].plot(ts, drx, label="standard")
ax[3].plot(ts, drx_clip, label="clipped")
ax[3].plot(ts, np.zeros_like(drx), linestyle="--", color="gray", linewidth=0.75)
ax[3].set_ylabel("$\Delta r_x$")

ax[4].plot(ts, dry, label="standard")
ax[4].plot(ts, dry_clip, label="clipped")
ax[4].plot(ts, np.zeros_like(dry), linestyle="--", color="gray", linewidth=0.75)
ax[4].set_ylabel("$\Delta r_y$")

ax[5].plot(ts, drz, label="standard")
ax[5].plot(ts, drz_clip, label="clipped")
ax[5].plot(ts, np.zeros_like(drz), linestyle="--", color="gray", linewidth=0.75)
ax[5].set_ylabel("$\Delta r_z$")

plt.show()

# %%

print("Going back home.")
robot.home()

# %%
robot.shutdown()

# %%
