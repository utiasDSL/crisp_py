"""Try to follow a "figure eight" target on the yz plane and save trajectory to numpy arrays."""

from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np

from crisp_py.robot import Robot

try:
    from rich import print
except ImportError:
    print = print


def main():
    """Main function to execute the figure eight trajectory experiment."""
    print("> Figure Eight Trajectory Experiment")

    # Ask for output filename
    default_filename = f"figure_eight_trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    output_filename = input(f"Enter output filename (default: {default_filename}): ").strip()
    if not output_filename:
        output_filename = default_filename

    # Ask for controller configuration
    print("\nAvailable controller configurations:")
    print("1. config/control/default_cartesian_impedance.yaml (default)")
    print("2. config/control/gravity_compensation.yaml")
    print("3. config/control/default_operational_space_controller.yaml")
    print("4. config/control/clipped_cartesian_impedance.yaml")
    print("5. Custom path")

    choice = input("Select controller (1-5, default: 1): ").strip()

    controller_configs = {
        "1": "config/control/default_cartesian_impedance.yaml",
        "2": "config/control/gravity_compensation.yaml",
        "3": "config/control/default_operational_space_controller.yaml",
        "4": "config/control/clipped_cartesian_impedance.yaml",
    }

    if choice in controller_configs:
        controller_path = controller_configs[choice]
    elif choice == "5":
        controller_path = input("Enter custom controller path: ").strip()
    else:
        controller_path = controller_configs["1"]  # default

    print(f"\nUsing controller: {controller_path}")

    # Initialize robot
    left_arm = Robot(namespace="left")
    left_arm.wait_until_ready()

    print(f"Initial end effector pose: {left_arm.end_effector_pose}")
    print(f"Initial joint values: {left_arm.joint_values}")

    # Go to home position
    print("Going to home position...")
    left_arm.home()

    # Parameters for the figure eight
    radius = 0.2  # [m]
    center = np.array([0.4, 0.0, 0.4])
    ctrl_freq = 50.0
    sin_freq_y = 0.25  # rot / s
    sin_freq_z = 0.125  # rot / s
    max_time = 8.0

    # Switch to cartesian impedance controller
    left_arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")
    left_arm.cartesian_controller_parameters_client.load_param_config(file_path=controller_path)

    # Move to center position
    left_arm.move_to(position=center, speed=0.15)

    # Data collection arrays
    ee_poses = []
    target_poses = []
    ts = []

    print("Starting to draw a figure eight...")
    t = 0.0
    target_pose = left_arm.end_effector_pose.copy()
    rate = left_arm.node.create_rate(ctrl_freq)

    # Main trajectory loop
    while t < max_time:
        x = center[0]
        y = radius * np.sin(2 * np.pi * sin_freq_y * t) + center[1]
        z = radius * np.sin(2 * np.pi * sin_freq_z * t) + center[2]
        target_pose.position = np.array([x, y, z])

        left_arm.set_target(pose=target_pose)

        rate.sleep()

        ee_poses.append(left_arm.end_effector_pose.copy())
        target_poses.append(left_arm.target_pose.copy())
        ts.append(t)

        t += 1.0 / ctrl_freq

    # Settling time
    while t < max_time + 1.0:
        rate.sleep()

        ee_poses.append(left_arm.end_effector_pose.copy())
        target_poses.append(left_arm.target_pose.copy())
        ts.append(t)

        t += 1.0 / ctrl_freq

    print("Done drawing a figure eight!")

    # Prepare data as numpy arrays
    timestamps = np.array(ts)

    # End effector data
    ee_positions = np.array([pose.position for pose in ee_poses])
    ee_orientations = np.array([pose.orientation for pose in ee_poses])

    # Target data
    target_positions = np.array([pose.position for pose in target_poses])
    target_orientations = np.array([pose.orientation for pose in target_poses])

    # Save as numpy arrays
    np.savez(
        f"./output/{output_filename}.npz",
        timestamps=timestamps,
        ee_positions=ee_positions,
        ee_orientations=ee_orientations,
        target_positions=target_positions,
        target_orientations=target_orientations,
        controller_path=controller_path,
        experiment_params={
            "radius": radius,
            "center": center,
            "ctrl_freq": ctrl_freq,
            "sin_freq_y": sin_freq_y,
            "sin_freq_z": sin_freq_z,
            "max_time": max_time,
        },
    )

    print(f"\nTrajectory data saved to: {output_filename}.npz")
    print(f"Controller used: {controller_path}")
    print(f"Total data points: {len(timestamps)}")
    print(f"Experiment duration: {max_time + 1.0:.1f} seconds")

    # Create quick visualization
    y_ee = ee_positions[:, 1]
    z_ee = ee_positions[:, 2]
    y_t = target_positions[:, 1]
    z_t = target_positions[:, 2]

    fig, ax = plt.subplots(1, 2, figsize=(10, 5))
    ax[0].plot(y_ee, z_ee, label="current")
    ax[0].plot(y_t, z_t, label="target", linestyle="--")
    ax[0].set_xlabel("$y$")
    ax[0].set_ylabel("$z$")
    ax[0].legend()
    ax[0].set_title("Figure Eight Trajectory")

    ax[1].plot(timestamps, z_ee, label="current")
    ax[1].plot(timestamps, z_t, label="target", linestyle="--")
    ax[1].set_xlabel("$t$")
    ax[1].set_ylabel("$z$")
    ax[1].legend()
    ax[1].set_title("Z Position vs Time")

    for a in ax:
        a.grid()

    fig.tight_layout()
    plt.show()

    print("Going back home.")
    left_arm.home()

    left_arm.shutdown()


if __name__ == "__main__":
    main()

