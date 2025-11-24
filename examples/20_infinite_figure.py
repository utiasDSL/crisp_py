"""Try to follow a "figure eight" target on the yz plane."""

import numpy as np

from crisp_py.robot import Robot

left_arm = Robot()
left_arm.wait_until_ready()

# %%
print(left_arm.end_effector_pose)
print(left_arm.joint_values)

# %%
print("Going to home position...")
left_arm.home()
homing_pose = left_arm.end_effector_pose.copy()


# %%
# Parameters for the circle
radius = 0.1  # [m]
center = np.array([0.4, 0.0, 0.4])
ctrl_freq = 50.0
sin_freq_y = 0.25  # rot / s
sin_freq_z = 0.125  # rot / s
max_time = 8.0

# %%
left_arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")
left_arm.cartesian_controller_parameters_client.load_param_config(
    # file_path="config/control/gravity_compensation.yaml"
    # file_path="config/control/default_operational_space_controller.yaml"
    # file_path="config/control/clipped_cartesian_impedance.yaml"
    file_path="config/control/default_cartesian_impedance.yaml"
)

# %%
# The move_to function will publish a pose to /target_pose while interpolation linearly
left_arm.move_to(position=center, speed=0.15)


print("Starting to draw a circle...")
target_pose = left_arm.end_effector_pose.copy()
rate = left_arm.node.create_rate(ctrl_freq)

try:
    while True:
        t = 0.0
        n_steps = 0

        while t < max_time:
            x = center[0]
            y = radius * np.sin(2 * np.pi * sin_freq_y * t) + center[1]
            z = radius * np.sin(2 * np.pi * sin_freq_z * t) + center[2]
            target_pose.position = np.array([x, y, z])
            left_arm.set_target(pose=target_pose)
            rate.sleep()
            t += 1.0 / ctrl_freq
            n_steps += 1
        
        print(f"Completed {n_steps} steps in {t:.2f} seconds corresponding to frequency {n_steps / t:.2f} Hz")

except KeyboardInterrupt:
    print("Interrupted by user.")
    print("Going back home.")
    left_arm.home()
    left_arm.shutdown()
