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
params = [
    ("task.k_pos_x", 400.0),
    ("task.k_pos_y", 400.0),
    ("task.k_pos_z", 400.0),
    ("task.k_rot_x", 10.0),
    ("task.k_rot_y", 10.0),
    ("task.k_rot_z", 10.0),
    ("nullspace.stiffness", 1.0),
    ("nullspace.projector_type", "kinematic"),
]
robot.cartesian_controller_parameters_client.set_parameters(params)

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
params = [
    ("task.k_pos_x", 0.0),
    ("task.k_pos_y", 0.0),
    ("task.k_pos_z", 0.0),
    ("task.k_rot_x", 0.0),
    ("task.k_rot_y", 0.0),
    ("task.k_rot_z", 0.0),
    ("nullspace.stiffness", 5.0),
    ("nullspace.projector_type", "none"),
]
robot.cartesian_controller_parameters_client.set_parameters(params)
