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

input("Press Enter to move to a new target pose... WARNING: ONLY USE FOR UR16, CHECK THE POSITION BEFORE PRESSING! (or change the position in the code)")
# WARNING: the folowing position has been chosen for the UR16!
target_position = np.array([0.0, 0.24, 0.75])
target_orientation = robot.end_effector_pose.orientation # Keep the same orientation

new_target_pose = Pose(position=target_position, orientation=target_orientation)
robot.move_to(pose=new_target_pose, speed=0.3)  

# This would publish this target directly to the controller, without any interpolation. Use with care!
# robot.set_target(pose=new_target_pose)

# %% 

print(f"Shutting down connection in 4 seconds... (robot will stay in place).")
time.sleep(4.0)

robot.shutdown()
