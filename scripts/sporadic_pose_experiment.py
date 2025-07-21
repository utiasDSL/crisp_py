"""Sporadic pose experiment with dual arm setup - save data to numpy arrays."""

import os
from datetime import datetime

import numpy as np
from scipy.spatial.transform import Rotation

from crisp_py.robot import Robot

try:
    from rich import print
except ImportError:
    print = print


def get_error(target_poses, ee_poses):
    """Get the error between the end effector and the target pose."""
    # Extract position data
    target_positions = np.array([pose.position for pose in target_poses])
    ee_positions = np.array([pose.position for pose in ee_poses])

    # Extract rotation data as rotation matrices
    target_rotations = np.array([pose.orientation.as_matrix() for pose in target_poses])
    ee_rotations = np.array([pose.orientation.as_matrix() for pose in ee_poses])

    # Compute position errors
    position_errors = ee_positions - target_positions
    dx, dy, dz = position_errors[:, 0], position_errors[:, 1], position_errors[:, 2]

    # Compute rotation errors using axis-angle representation
    # R_error = R_ee @ R_t^T gives the relative rotation from target to current
    R_error = ee_rotations @ target_rotations.transpose(0, 2, 1)

    # Convert to axis-angle (rotvec) representation
    d_rot = np.array([Rotation.from_matrix(R).as_rotvec() for R in R_error])
    drx = d_rot[:, 0]
    dry = d_rot[:, 1]
    drz = d_rot[:, 2]

    return dx, dy, dz, drx, dry, drz


def drive_trajectory(robot, target_pose, max_time=10.0, ctrl_freq=100.0):
    """Drive robot to target pose and collect trajectory data."""
    ee_poses = []
    target_poses = []
    ts = []

    rate = robot.node.create_rate(ctrl_freq)

    t = 0.0
    while t < max_time:
        if t > 1.0:  # Start moving after 1 second
            robot.set_target(pose=target_pose)

        rate.sleep()

        ee_poses.append(robot.end_effector_pose.copy())
        target_poses.append(robot.target_pose.copy())
        ts.append(t)

        t += 1.0 / ctrl_freq

    return ts, ee_poses, target_poses


def sample_random_pose(robot, min_distance=0.3, max_distance=0.7):
    """Sample a random reachable pose within workspace limits."""
    # Get current home pose as reference
    home_pose = robot.end_effector_pose.copy()

    # Sample random position within sphere around home
    while True:
        # Sample spherical coordinates
        radius = np.random.uniform(min_distance, max_distance)
        theta = np.random.uniform(0, 2 * np.pi)  # azimuth
        phi = np.random.uniform(0, np.pi)  # elevation

        # Convert to Cartesian offset
        dx = radius * np.sin(phi) * np.cos(theta)
        dy = radius * np.sin(phi) * np.sin(theta)
        dz = radius * np.cos(phi)

        # Apply constraints to keep within reasonable workspace
        new_position = home_pose.position + np.array([dx, dy, dz])

        # Check basic workspace limits (adjust as needed for your robot)
        if (
            0.2 <= new_position[0] <= 0.8
            and -0.6 <= new_position[1] <= 0.6
            and 0.1 <= new_position[2] <= 0.8
        ):
            break

    # Keep original orientation with small random perturbation
    target_pose = home_pose.copy()
    target_pose.position = new_position

    # Add small random orientation perturbation (±10 degrees)
    perturbation = np.random.uniform(-0.175, 0.175, 3)  # ±10 degrees in radians
    R_perturbation = Rotation.from_rotvec(perturbation)
    target_pose.orientation = target_pose.orientation * R_perturbation

    return target_pose


def main():
    """Main function to execute the sporadic pose experiment."""
    print("> Sporadic Pose Experiment")

    # Get user input for experiment parameters
    default_filename = f"sporadic_pose_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    output_filename = input(f"Enter output filename (default: {default_filename}): ").strip()
    if not output_filename:
        output_filename = default_filename

    # Ask for pose selection method
    print("\nTarget pose selection:")
    print("1. Manual positioning with left arm")
    print("2. Random pose generation")

    pose_choice = input("Select method (1-2, default: 1): ").strip()
    use_random_pose = pose_choice == "2"

    # Ask for controller configuration
    print("\nController selection mode:")
    print("1. Single controller")
    print("2. Multiple controllers (sequential comparison)")

    controller_mode = input("Select mode (1-2, default: 1): ").strip()
    use_multiple_controllers = controller_mode == "2"

    controller_configs = {
        "1": "config/control/default_cartesian_impedance.yaml",
        "2": "config/control/default_operational_space_controller.yaml",
        "3": "config/control/clipped_cartesian_impedance.yaml",
    }

    if use_multiple_controllers:
        print("\nAvailable controller configurations:")
        print("1. config/control/default_cartesian_impedance.yaml")
        print("2. config/control/default_operational_space_controller.yaml")
        print("3. config/control/clipped_cartesian_impedance.yaml")
        print("4. All controllers (1,2,3)")
        print("5. Custom selection")

        choice = input("Select controllers (1-5, default: 4): ").strip()

        if choice == "4" or choice == "":
            selected_controllers = list(controller_configs.values())
        elif choice in controller_configs:
            selected_controllers = [controller_configs[choice]]
        elif choice == "5":
            print("Enter controller numbers separated by commas (e.g., '1,3'):")
            custom_choice = input().strip()
            controller_indices = [x.strip() for x in custom_choice.split(",")]
            selected_controllers = [
                controller_configs[idx] for idx in controller_indices if idx in controller_configs
            ]
        else:
            selected_controllers = list(controller_configs.values())  # default to all

        print(f"\nUsing controllers: {selected_controllers}")
    else:
        print("\nAvailable controller configurations:")
        print("1. config/control/default_cartesian_impedance.yaml (default)")
        print("2. config/control/default_operational_space_controller.yaml")
        print("3. config/control/clipped_cartesian_impedance.yaml")
        print("4. Custom path")

        choice = input("Select controller (1-4, default: 1): ").strip()

        if choice in controller_configs:
            controller_path = controller_configs[choice]
        elif choice == "4":
            controller_path = input("Enter custom controller path: ").strip()
        else:
            controller_path = controller_configs["1"]  # default

        selected_controllers = [controller_path]
        print(f"\nUsing controller: {controller_path}")

    # Initialize robots
    if use_random_pose:
        print("Initializing right arm only...")
        right_arm = Robot(namespace="right")
        right_arm.wait_until_ready()
        left_arm = None
    else:
        print("Initializing both arms...")
        left_arm = Robot(namespace="left")
        right_arm = Robot(namespace="right")
        left_arm.wait_until_ready()
        right_arm.wait_until_ready()

    # Go to home positions
    print("Going to home positions...")
    if left_arm:
        left_arm.home()
    right_arm.home()

    # Get target pose
    if use_random_pose:
        print("Generating random target pose...")
        target_pose = sample_random_pose(right_arm)
        print(f"Target position: {target_pose.position}")
        print(f"Target orientation (quaternion): {target_pose.orientation.as_quat()}")
    else:
        # Set up left arm for manual positioning
        left_arm.cartesian_controller_parameters_client.load_param_config(
            file_path="config/control/gravity_compensation.yaml"
        )
        left_arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")

        print("Move the left arm to a desired pose, press enter after choosing your target pose.")
        input()
        target_pose = left_arm.end_effector_pose.copy()

    print("Now the right arm will drive to that pose.")
    if not use_random_pose:
        print("Press enter to continue...")
        input()

    # Execute trajectory for each controller
    max_time = 8.0
    ctrl_freq = 100.0

    # Create output directory if it doesn't exist
    os.makedirs("output", exist_ok=True)

    all_results = []

    for i, controller_path in enumerate(selected_controllers):
        controller_name = os.path.basename(controller_path).replace(".yaml", "")

        print(f"\n=== Controller {i + 1}/{len(selected_controllers)}: {controller_name} ===")

        right_arm.home()
        # Configure right arm controller
        right_arm.cartesian_controller_parameters_client.load_param_config(
            file_path=controller_path
        )
        right_arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")

        print(f"Executing trajectory for {max_time} seconds...")
        ts, ee_poses, target_poses = drive_trajectory(
            right_arm, target_pose, max_time=max_time, ctrl_freq=ctrl_freq
        )

        # Compute errors
        dx, dy, dz, drx, dry, drz = get_error(target_poses, ee_poses)

        # Prepare data as numpy arrays
        timestamps = np.array(ts)

        # End effector data
        ee_positions = np.array([pose.position for pose in ee_poses])
        ee_orientations = np.array([pose.orientation.as_quat() for pose in ee_poses])
        ee_rotations = np.array([pose.orientation.as_matrix() for pose in ee_poses])

        # Target data
        target_positions = np.array([pose.position for pose in target_poses])
        target_orientations = np.array([pose.orientation.as_quat() for pose in target_poses])
        target_rotations = np.array([pose.orientation.as_matrix() for pose in target_poses])

        # Error data
        position_errors = np.column_stack([dx, dy, dz])
        orientation_errors = np.column_stack([drx, dry, drz])

        # Position and orientation error magnitudes
        position_error_magnitude = np.sqrt(dx**2 + dy**2 + dz**2)
        orientation_error_magnitude = np.sqrt(drx**2 + dry**2 + drz**2)

        # Generate filename for this controller
        if use_multiple_controllers:
            controller_suffix = f"_{controller_name}"
            current_filename = f"{output_filename}{controller_suffix}"
        else:
            current_filename = output_filename

        # Save as numpy arrays
        np.savez(
            f"output/{current_filename}.npz",
            timestamps=timestamps,
            ee_positions=ee_positions,
            ee_orientations=ee_orientations,
            ee_rotations=ee_rotations,
            target_positions=target_positions,
            target_orientations=target_orientations,
            target_rotations=target_rotations,
            position_errors=position_errors,
            orientation_errors=orientation_errors,
            position_error_magnitude=position_error_magnitude,
            orientation_error_magnitude=orientation_error_magnitude,
            controller_path=controller_path,
            experiment_params={
                "max_time": max_time,
                "ctrl_freq": ctrl_freq,
                "target_pose_position": target_pose.position,
                "target_pose_orientation": target_pose.orientation.as_quat(),
                "use_random_pose": use_random_pose,
                "controller_name": controller_name,
            },
        )

        print(f"Experiment data saved to: output/{current_filename}.npz")
        print(f"Final position error: {position_error_magnitude[-1] * 1000:.2f} mm")
        print(f"Final orientation error: {orientation_error_magnitude[-1]:.4f} rad")

        # Store results for summary
        all_results.append(
            {
                "controller": controller_name,
                "filename": f"{current_filename}.npz",
                "final_pos_error": position_error_magnitude[-1] * 1000,
                "final_rot_error": orientation_error_magnitude[-1],
                "rms_pos_error": np.sqrt(np.mean(position_error_magnitude**2)) * 1000,
                "rms_rot_error": np.sqrt(np.mean(orientation_error_magnitude**2)),
            }
        )

    # Print summary for multiple controllers
    if use_multiple_controllers:
        print("\n=== EXPERIMENT SUMMARY ===")
        print(f"Target pose: {target_pose.position}")
        print(f"Total controllers tested: {len(selected_controllers)}")
        print(f"Experiment duration per controller: {max_time:.1f} seconds")
        print(f"Total data points per controller: {len(timestamps)}")
        print("\nController Comparison:")
        print(
            f"{'Controller':<35} {'Final Pos [mm]':<15} {'Final Rot [rad]':<15} {'RMS Pos [mm]':<15} {'RMS Rot [rad]':<15}"
        )
        print("-" * 95)
        for result in all_results:
            print(
                f"{result['controller']:<35} {result['final_pos_error']:<15.2f} {result['final_rot_error']:<15.4f} {result['rms_pos_error']:<15.2f} {result['rms_rot_error']:<15.4f}"
            )
    else:
        print(f"\nController used: {selected_controllers[0]}")
        print(f"Total data points: {len(timestamps)}")
        print(f"Experiment duration: {max_time:.1f} seconds")

    # Return to home
    print("Going back home.")
    right_arm.home()
    if left_arm:
        left_arm.home()

    # Shutdown
    right_arm.shutdown()
    if left_arm:
        left_arm.shutdown()

    # Return list of generated files
    if use_multiple_controllers:
        return [result["filename"] for result in all_results]
    else:
        return f"output/{output_filename}.npz"


if __name__ == "__main__":
    main()
