import numpy as np
from scipy.spatial.transform import Rotation

# %%
from crisp_py.robot import Pose, Robot
from crisp_py.robot_config import IiwaConfig

robot = Robot(robot_config=IiwaConfig())
robot.wait_until_ready()

# %%
print(robot.end_effector_pose)

# %%
print("Going to home position...")
robot.home()

# %%

params = [
    ("task.k_pos_x", 2500.0),
    ("task.k_pos_y", 2500.0),
    ("task.k_pos_z", 2500.0),
    ("task.k_rot_x", 80.0),
    ("task.k_rot_y", 80.0),
    ("task.k_rot_z", 80.0),
    ("nullspace.stiffness", 5.0),
    ("nullspace.weights.joint_a1.value", 5.0),
    ("nullspace.weights.joint_a2.value", 5.0),
    ("nullspace.weights.joint_a3.value", 5.0),
    ("nullspace.weights.joint_a4.value", 2.0),
    ("nullspace.weights.joint_a5.value", 2.0),
    ("nullspace.weights.joint_a6.value", 1.0),
    ("nullspace.weights.joint_a7.value", 1.0),
]

robot.cartesian_controller_parameters_client.set_parameters(params)
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

# %%
# Paremeters for the circle
radius = 0.1  # [m]
center = [0.5, 0.0, 0.2]
ctrl_freq = 50.0
sin_freq = 0.25  # rot / s
max_time = 10.0

# %%
# The move_to function will publish a pose to /target_pose while interpolation linearly
starting_pose = Pose(
    position=np.array(center) + np.array([radius * np.cos(0.0), radius * np.sin(0.0), 0]),
    orientation=Rotation.from_euler("xyz", [-180, 0, -180], degrees=True),
)

robot.move_to(pose=starting_pose, speed=0.15)

# %%
# The set_target will directly publish the pose to /target_pose

print("Starting to draw a circle...")
t = 0.0
target_pose = robot.end_effector_pose.copy()
rate = robot.node.create_rate(ctrl_freq)

while t < max_time:
    x = radius * np.cos(2 * np.pi * sin_freq * t) + center[0]
    y = radius * np.sin(2 * np.pi * sin_freq * t) + center[1]
    z = center[2]
    target_pose.position = np.array([x, y, z])

    robot.set_target(pose=target_pose)
    rate.sleep()

    t += 1.0 / ctrl_freq


print("Done drawing a circle!")

# %%
print("Going back home.")
robot.home()

# %%
robot.shutdown()

# %%
