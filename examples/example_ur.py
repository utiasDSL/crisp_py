"""Example script for using crisp_py with a UR robot. This example assumes you have the `ur_robot_driver` ROS package running and properly configured to control your UR robot."""
import time
import numpy as np

from crisp_py.robot import make_robot
from crisp_py.utils.geometry import Pose

robot = make_robot("ur")
robot.wait_until_ready()

#%%

print(f"Starting pose: {robot.end_effector_pose}")
print(f"Starting joint values: {robot.joint_values}")

#%%

print("Going to home position...")
robot.home()  # This requires the joint_trajectory_controller to be active
homing_pose = robot.end_effector_pose.copy()

print(f"Homing pose: {homing_pose}")

#%%

print("Switching to Cartesian Impedance Controller...")
print("This will unload other controllers if necessary.")

# Change parameters now if needed
# robot.cartesian_controller_parameters_client.set_parameters([
#     ("task.k_pos_x", 600.0),
#     ...
# ])
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

#%%

# Figure eight
input("Press Enter to start figure eight trajectory...")

duration = 6.0
start_time = time.time()
rate = 20.0

while time.time() - start_time < duration:
    t = time.time() - start_time

    # Create figure eight trajectory in the XZ plane
    x = 0.1 * np.sin(2 * np.pi * t / duration)
    y = 0.0
    z = 0.1 * np.sin(4 * np.pi * t / duration)

    target_pose = Pose(
        position=homing_pose.position + np.array([x, y, z]),
        orientation=homing_pose.orientation,
    )

    robot.set_target(pose=target_pose)

    time.sleep(1.0 / rate)


# %%

print(f"Shutting down connection in 4 seconds... (robot will stay in place).")
time.sleep(4.0)

robot.shutdown()

