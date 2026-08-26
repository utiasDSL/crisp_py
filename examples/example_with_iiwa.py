"""Follow a figure eight on a KUKA LBR iiwa driven through lbr_fri_ros2_stack.

Start the bringup first, with the arm in a test mode and the enabling switch held.

Unlike the numbered examples this one loads no profile from config/control. Those are
tuned for a Franka, and this arm is not one: FRI applies the torque overlay on top of
the cabinet's own joint impedance controller, so the achievable task stiffness is
bounded by that, and the model compensation is already being done underneath. Leave
the controller with whatever the bringup configured.
"""

# %%
import matplotlib.pyplot as plt
import numpy as np

from crisp_py.robot import Robot, make_robot

robot = make_robot("iiwa14_r820")
robot.wait_until_ready()

# %%
print(robot.end_effector_pose)
print(robot.joint_values)


# %%
def home(robot: Robot) -> None:
    """Home the arm, keeping the torque overlay owned throughout.

    robot.home() cannot be used here. It switches through the stock switcher, which
    deactivates every active controller not ending in "broadcaster", and offers no way
    to exempt one.

    That matters because command interface values persist: the hardware reads the last
    value written to an interface whether or not a controller still owns it, and
    lbr_ros2_control NaNs its command interfaces only once, when the hardware component
    activates. So dropping the Cartesian controller without handing the effort
    interfaces to something leaves its final torques, tens of Nm, applied for the whole
    homing move. zero_effort_controller keeps writing an actual zero.
    """
    robot.controller_switcher_client.switch_controller(
        "joint_trajectory_controller",
        controllers_that_should_be_active=[
            "zero_effort_controller",
            "estimated_wrench_interface",
        ],
    )
    robot.joint_trajectory_controller_client.send_joint_config(
        robot.config.joint_names,
        robot.config.home_config,
        robot.config.time_to_home,
        blocking=True,
    )


print("Going to home position...")
home(robot)
home_pose = robot.end_effector_pose.copy()
print(f"home pose: {home_pose.position}")

# %%
# The figure is centred on wherever homing actually left the tool rather than on a
# hardcoded point, so the arm never has to traverse the workspace to start, and the
# orientation is left at the homing one: commanding an unrelated orientation turns the
# first move into a large rotation.
center = home_pose.position.copy()
radius = 0.1  # [m]
ctrl_freq = 50.0
sin_freq_y = 0.25  # rot / s
sin_freq_z = 0.125  # rot / s
max_time = 8.0

# %%
# fri_position_passthrough_controller writes the measured joint positions into the
# position command every cycle. If the switcher drops it, that command freezes at its
# last value while the arm keeps moving under the torque overlay, and the cabinet's
# joint impedance pulls against a setpoint that grows more wrong as the arm moves.
# estimated_wrench_interface only feeds force_torque_broadcaster; it is named so a
# BEST_EFFORT partial switch cannot silently cost the wrench topic.
robot.controller_switcher_client.switch_controller(
    "cartesian_impedance_controller",
    controllers_that_should_be_active=[
        "fri_position_passthrough_controller",
        "estimated_wrench_interface",
    ],
)

# %%
ee_poses = []
target_poses = []
ts = []

print("Starting to draw a figure eight...")
t = 0.0
target_pose = robot.end_effector_pose.copy()
rate = robot.node.create_rate(ctrl_freq)

while t < max_time:
    target_pose.position = np.array(
        [
            center[0],
            radius * np.sin(2 * np.pi * sin_freq_y * t) + center[1],
            radius * np.sin(2 * np.pi * sin_freq_z * t) + center[2],
        ]
    )

    robot.set_target(pose=target_pose)

    rate.sleep()

    ee_poses.append(robot.end_effector_pose.copy())
    target_poses.append(robot.target_pose.copy())
    ts.append(t)

    t += 1.0 / ctrl_freq

while t < max_time + 1.0:
    # Let the arm settle. Expect it to keep moving after the last target: the overlay
    # is soft, so the tool lags the target by a visible margin.
    rate.sleep()

    ee_poses.append(robot.end_effector_pose.copy())
    target_poses.append(robot.target_pose.copy())
    ts.append(t)

    t += 1.0 / ctrl_freq

print("Done drawing a figure eight!")

# %%
y_t = [pose.position[1] for pose in target_poses]
z_t = [pose.position[2] for pose in target_poses]
y_ee = [pose.position[1] for pose in ee_poses]
z_ee = [pose.position[2] for pose in ee_poses]

# %%
fig, ax = plt.subplots(1, 2, figsize=(10, 5))
ax[0].plot(y_ee, z_ee, label="current")
ax[0].plot(y_t, z_t, label="target", linestyle="--")
ax[0].set_xlabel("$y$")
ax[0].set_ylabel("$z$")
ax[0].set_aspect("equal")
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
# The case the helper exists for: coming straight out of Cartesian impedance, so there
# are real torques in the effort command interfaces to be replaced with zero.
home(robot)

# %%
robot.shutdown()
