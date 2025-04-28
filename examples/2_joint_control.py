"""Example controlling the joints."""

import numpy as np

from crisp_py.robot import Robot

robot = Robot(namespace="right")
robot.wait_until_ready()

# %%
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

# %%
print(robot.cartesian_controller_parameters_client.list_parameters())

# %%
params_names = [
    "task.k_pos_x",
    "task.k_pos_y",
    "task.k_pos_z",
    "task.k_rot_x",
    "task.k_rot_y",
    "task.k_rot_z",
    "nullspace.stiffness",
    "nullspace.projector_type",
]
params_values = [
    500.0,
    500.0,
    500.0,
    20.0,
    20.0,
    20.0,
    5.0,
    "kinematic",
]
robot.cartesian_controller_parameters_client.set_parameters(params_names, params_values)

# %%
max_time = 8.0
sin_freq = 0.125  # rot / s
ctrl_freq = 50.0
amplitude = np.pi / 2  # rad

target_q = robot.joint_values.copy()

t = 0.0
rate = robot.node.create_rate(ctrl_freq)

while t < max_time:
    target_q[0] = amplitude * np.sin(2 * np.pi * sin_freq * t)
    robot.set_target_joint(target_q)

    rate.sleep()
    t += 1.0 / ctrl_freq

# %%
# Or archive pure joint control with:
params_names = [
    "task.k_pos_x",
    "task.k_pos_y",
    "task.k_pos_z",
    "task.k_rot_x",
    "task.k_rot_y",
    "task.k_rot_z",
    "nullspace.stiffness",
    "nullspace.projector_type",
]
params_values = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    3.0,
    "none",
]
robot.cartesian_controller_parameters_client.set_parameters(params_names, params_values)
