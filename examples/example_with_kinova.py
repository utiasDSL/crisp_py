import numpy as np
from scipy.spatial.transform import Rotation

# %%
from crisp_py.robot import Robot, Pose
from crisp_py.robot_config import KinovaConfig

robot = Robot(robot_config=KinovaConfig())
robot.wait_until_ready()

# %%
print(robot.end_effector_pose)

# %%
print("Going to home position...")
robot.home()

robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

# %%
# Paremeters for the circle
radius = 0.1  # [m]
center = [0.5, 0.0, 0.3]
ctrl_freq = 50.0
sin_freq = 0.25  # rot / s
max_time = 10.0

# %%
# The move_to function will publish a pose to /target_pose while interpolation linearly
starting_pose = Pose(
    orientation=robot.end_effector_pose.orientation,
    position=np.array(center) + np.array([radius * np.cos(0.0), radius * np.sin(0.0), 0]),
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
