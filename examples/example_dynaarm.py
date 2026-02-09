"""Example script to control the DynaArm robot using crisp_py."""
from pathlib import Path
import time
import numpy as np

from crisp_py.robot import make_robot

robot = make_robot("dynaarm")
robot.wait_until_ready()  # Wait until the robot is ready to receive commands

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

# path = Path("config/control/dynaarm_gravity.yaml")
path = Path("config/control/dynaarm_cic_soft.yaml")

robot.cartesian_controller_parameters_client.load_param_config(path)
# robot.cartesian_controller_parameters_client.save_param_config("config/control/dynaarm_cic_soft.yaml")

robot.controller_switcher_client.switch_controller(
    "crisp_cartesian_controller", 
    controllers_that_should_be_activate=["freedrive_controller", "safety_monitor_controller"]
)

robot.reset_targets()
#%%

target_pose = robot.end_effector_pose
target_pose.position[2] -= 0.1

#%%

# we apply a sine wave in z direction for 10 seconds from -0.15 to 0.15 m

ee_poses = []
target_poses = []
ts = []

start_time = time.time()
while True:
    t = time.time()
    position = target_pose.position.copy()
    # position[0] = target_pose.position[0] + 0.2 * np.sin(2 * np.pi * 0.6 * (t - start_time))  # 0.1 Hz sine wave
    position[2] = target_pose.position[2] + 0.15 * np.sin(2 * np.pi * 0.3 * (t - start_time))  # 0.05 Hz sine wave
    robot.set_target(position=position)
    if t - start_time > 10.0:
        break
    ee_poses.append(robot.end_effector_pose.copy())
    target_poses.append(robot.target_pose.copy())
    ts.append(t - start_time)
    time.sleep(0.01)  # Sleep for a short time to avoid busy waiting


#%%

import matplotlib.pyplot as plt
x_t = [target_pose_sample.position[0] for target_pose_sample in target_poses]
y_t = [target_pose_sample.position[1] for target_pose_sample in target_poses]
z_t = [target_pose_sample.position[2] for target_pose_sample in target_poses]

x_ee = [ee_pose.position[0] for ee_pose in ee_poses]
y_ee = [ee_pose.position[1] for ee_pose in ee_poses]
z_ee = [ee_pose.position[2] for ee_pose in ee_poses]

fig, ax = plt.subplots(1, 2, figsize=(10, 5))
ax[0].plot(ts, x_ee, label="current")
ax[0].plot(ts, x_t, label="target", linestyle="--")
ax[0].set_ylabel("$x$")
ax[0].set_xlabel("$t$")
ax[0].legend()

ax[1].plot(ts, z_ee, label="current")
ax[1].plot(ts, z_t, label="target", linestyle="--")
ax[1].set_ylabel("$z$")
ax[1].set_xlabel("$t$")
ax[1].legend()


for a in ax:
    a.grid()

fig.tight_layout()

plt.show()
#%%
print(f"End pose: {robot.end_effector_pose}")
print(f"End joint values: {robot.joint_values}")

#%%

print(f"Shutting down connection in 2 seconds... (robot will stay in place).")
time.sleep(2.0)

robot.shutdown()

# %%

robot.controller_switcher_client.switch_controller(
    "gravity_compensation_controller",
    controllers_that_should_be_activate=["freedrive_controller", "safety_monitor_controller"]
)
