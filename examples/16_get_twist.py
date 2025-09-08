"""Example showing how to get the end-effector twist (linear and angular velocity) of the robot."""

# %%
import time

from rich import print
import numpy as np
import matplotlib.pyplot as plt

from crisp_py.robot import Robot
from crisp_py.utils.geometry import Pose

# %%

# robot = Robot()
robot = Robot(namespace="left")
robot.wait_until_ready()

# %%
print(robot.end_effector_twist)

# %%

def figure_eight(t: float, dt: float, amplitude: float = 0.2, omega: float = 0.01) -> np.ndarray:
    """Generate a figure eight trajectory on the yz plane."""
    y = amplitude * np.sin(omega * t / dt)
    z = amplitude * np.sin(omega * t / dt) * np.cos(omega * t / dt)
    return np.array([0.0, y, z])


robot.home()
robot.cartesian_controller_parameters_client.load_param_config(
    file_path="config/control/default_cartesian_impedance.yaml"
)
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")

start_pose = robot.end_effector_pose.copy()
start_time = time.time()
max_duration = 10.0  # seconds
dt = 0.01  # control loop time step

end_effector_z_position = []
end_effector_z_velocity = []

while (time.time() - start_time) < max_duration:
    t = time.time() - start_time
    target_offset = figure_eight(t, dt)

    target_position = start_pose.position + target_offset
    target_pose = Pose(
        position=target_position,
        orientation=start_pose.orientation,
    )
    robot.set_target(pose=target_pose)

    end_effector_z_position.append(robot.end_effector_pose.position[2])
    end_effector_z_velocity.append(robot.end_effector_twist.linear[2])

    time.sleep(dt)


# %%

plt.figure()
plt.subplot(2, 1, 1)
plt.plot(np.arange(len(end_effector_z_position)) * dt, end_effector_z_position)
plt.title("End Effector Z Position")
plt.xlabel("Time (s)")
plt.ylabel("Z Position (m)")
plt.grid()
plt.subplot(2, 1, 2)
plt.plot(np.arange(len(end_effector_z_velocity)) * dt, end_effector_z_velocity)
plt.title("End Effector Z Velocity")
plt.xlabel("Time (s)")
plt.ylabel("Z Velocity (m/s)")
plt.grid()
plt.tight_layout()
plt.show()


