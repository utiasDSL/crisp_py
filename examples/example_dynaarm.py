"""Example script to control the DynaArm robot using crisp_py."""
import time

from crisp_py.robot import make_robot

robot = make_robot("dynaarm")
robot.wait_until_ready()  # Wait until the robot is ready to receive commands

#%%

print(f"Starting pose: {robot.end_effector_pose}")
print(f"Starting joint values: {robot.joint_values}")

#%%

print("Going to home position...") 
robot.home()  # This requirese the joint_trajectory_controller to be active
homing_pose = robot.end_effector_pose.copy()

print(f"Homing pose: {homing_pose}")

#%%

print("Switching to Cartesian Impedance Controller...")
print("This will unload other controllers if necessary.")
robot.controller_switcher_client.switch_controller(
    "cartesian_impedance_controller", 
    controllers_that_should_be_activate=["free_drive_controller"]
)


#%%

current_pose = robot.end_effector_pose
current_pose.position += [0.1, 0.0, 0.0]

print(f"Moving to new pose: {current_pose} in 2.0 seconds")
time.sleep(2.0)
# robot.move_to(pose=current_pose, speed=0.1) to move with interpolation
robot.set_target(pose=current_pose)

#%%
print(f"End pose: {robot.end_effector_pose}")
print(f"End joint values: {robot.joint_values}")

#%%

print(f"Shutting down connection in 2 seconds... (robot will stay in place).")
time.sleep(2.0)

robot.shutdown()

