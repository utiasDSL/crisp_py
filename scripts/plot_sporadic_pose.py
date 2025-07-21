"""Plot sporadic pose experiment data with position and orientation error plots."""

import glob
import os
from datetime import datetime

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

try:
    from rich import print
    from rich.prompt import Prompt
except ImportError:
    print = print

mpl.rcParams.update(
    {
        "font.family": "serif",
        "font.size": 22,
        "axes.labelsize": 20,
        "legend.fontsize": 12,
        "xtick.labelsize": 18,
        "ytick.labelsize": 18,
        # "font.size": 22,
        # "axes.labelsize": 14,
        # "axes.titlesize": 14,
        # "legend.fontsize": 12,
        # "xtick.labelsize": 12,
        # "ytick.labelsize": 12,
        "text.usetex": True,
    }
)


def list_sporadic_pose_files(directory: str = "./output") -> list:
    """List all NPZ files containing sporadic pose experiments."""
    npz_files = glob.glob(os.path.join(directory, "*sporadic_pose*.npz"))
    return [os.path.basename(f) for f in npz_files]


def load_and_validate_sporadic_pose_data(filepath: os.PathLike) -> np.ndarray:
    """Load NPZ and validate it has the expected arrays for sporadic pose experiments."""
    data = np.load(filepath, allow_pickle=True)

    required_arrays = [
        "timestamps",
        "position_error_magnitude",
        "orientation_error_magnitude",
        "position_errors",
        "orientation_errors",
    ]

    missing_arrays = [arr for arr in required_arrays if arr not in data.files]
    if missing_arrays:
        raise ValueError(f"Missing required arrays: {', '.join(missing_arrays)}")

    return data


def plot_sporadic_pose_errors(data: np.ndarray, filename: str):
    """Create error plots for sporadic pose experiment."""
    timestamps = data["timestamps"]
    position_error_magnitude = data["position_error_magnitude"]
    orientation_error_magnitude = data["orientation_error_magnitude"]

    # Individual position and orientation errors
    position_errors = data["position_errors"]
    orientation_errors = data["orientation_errors"]

    dx, dy, dz = position_errors[:, 0], position_errors[:, 1], position_errors[:, 2]
    drx, dry, drz = orientation_errors[:, 0], orientation_errors[:, 1], orientation_errors[:, 2]

    # Create comprehensive error plot
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Position error magnitude
    axes[0, 0].plot(timestamps, position_error_magnitude * 1000, "b-", linewidth=2)
    axes[0, 0].set_ylabel(r"Position Error $e_{\mathrm{pos}}$ [mm]")
    axes[0, 0].set_title("Position Error Magnitude")
    axes[0, 0].grid(True, alpha=0.3)

    # Add final error annotation
    final_pos_error = position_error_magnitude[-1] * 1000
    axes[0, 0].annotate(
        f"Final: {final_pos_error:.2f} mm",
        xy=(timestamps[-1], final_pos_error),
        xytext=(timestamps[-1] - 2, final_pos_error + 5),
        fontsize=12,
        arrowprops=dict(arrowstyle="->", color="red", alpha=0.7),
    )

    # Orientation error magnitude
    axes[0, 1].plot(timestamps, orientation_error_magnitude, "r-", linewidth=2)
    axes[0, 1].set_ylabel(r"Orientation Error $e_{\mathrm{rot}}$ [rad]")
    axes[0, 1].set_title("Orientation Error Magnitude")
    axes[0, 1].grid(True, alpha=0.3)

    # Add final error annotation
    final_rot_error = orientation_error_magnitude[-1]
    axes[0, 1].annotate(
        f"Final: {final_rot_error:.4f} rad",
        xy=(timestamps[-1], final_rot_error),
        xytext=(timestamps[-1] - 2, final_rot_error + 0.01),
        fontsize=12,
        arrowprops=dict(arrowstyle="->", color="red", alpha=0.7),
    )

    # Individual position errors
    axes[1, 0].plot(timestamps, dx * 1000, label="$e_x$", linewidth=1.5)
    axes[1, 0].plot(timestamps, dy * 1000, label="$e_y$", linewidth=1.5)
    axes[1, 0].plot(timestamps, dz * 1000, label="$e_z$", linewidth=1.5)
    axes[1, 0].set_xlabel("Time [s]")
    axes[1, 0].set_ylabel("Position Error [mm]")
    axes[1, 0].set_title("Individual Position Errors")
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)

    # Individual orientation errors
    axes[1, 1].plot(timestamps, drx, label=r"$e_{r_x}$", linewidth=1.5)
    axes[1, 1].plot(timestamps, dry, label=r"$e_{r_y}$", linewidth=1.5)
    axes[1, 1].plot(timestamps, drz, label=r"$e_{r_z}$", linewidth=1.5)
    axes[1, 1].set_xlabel("Time [s]")
    axes[1, 1].set_ylabel("Orientation Error [rad]")
    axes[1, 1].set_title("Individual Orientation Errors")
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.suptitle(f"Sporadic Pose Error Analysis: {filename}", fontsize=18, y=0.98)

    # Save as PDF
    pdf_filename = f"sporadic_pose_errors_{filename.replace('.npz', '')}.pdf"
    plt.savefig(pdf_filename, bbox_inches="tight", dpi=300)
    print(f"Error analysis figure saved as: {pdf_filename}")

    # Print summary statistics
    print("\n=== Sporadic Pose Error Analysis ===")
    print(f"File: {filename}")
    print(f"Duration: {timestamps[-1] - timestamps[0]:.2f} seconds")
    print(f"Data points: {len(timestamps)}")

    # Print controller info if available
    if "controller_path" in data.files:
        controller_path = str(data["controller_path"])
        print(f"Controller used: {controller_path}")

    # Print experiment parameters if available
    if "experiment_params" in data.files:
        params = data["experiment_params"].item()
        print("Experiment parameters:")
        for key, value in params.items():
            if isinstance(value, np.ndarray):
                print(f"  {key}: {value}")
            else:
                print(f"  {key}: {value}")

    print("\nFinal Error Statistics:")
    print(f"  Position error: {position_error_magnitude[-1] * 1000:.2f} mm")
    print(f"  Orientation error: {orientation_error_magnitude[-1]:.4f} rad")
    print(f"  Position error RMS: {np.sqrt(np.mean(position_error_magnitude**2)) * 1000:.2f} mm")
    print(f"  Orientation error RMS: {np.sqrt(np.mean(orientation_error_magnitude**2)):.4f} rad")
    print(f"  Max position error: {np.max(position_error_magnitude) * 1000:.2f} mm")
    print(f"  Max orientation error: {np.max(orientation_error_magnitude):.4f} rad")

    plt.show()


def plot_comparison_sporadic_pose(datasets: list):
    """Create comparison plot for multiple sporadic pose experiments."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    for i, dataset in enumerate(datasets):
        data = dataset["data"]
        filename = dataset["filename"]

        timestamps = data["timestamps"]
        position_error_magnitude = data["position_error_magnitude"]
        orientation_error_magnitude = data["orientation_error_magnitude"]

        # Extract short identifier from filename
        short_name = filename.replace("sporadic_pose_", "").replace(".npz", "")
        controller_names = {
            "default_cartesian_impedance": "CI",
            "clipped_cartesian_impedance": "CI-clipped",
            "default_operational_space_controller": "OSC",
        }
        short_name = controller_names.get(short_name, short_name)

        # Plot position errors
        axes[0].plot(
            timestamps,
            position_error_magnitude * 1000,
            label=f"{short_name}",
            linewidth=2,
        )

        # Plot orientation errors
        axes[1].plot(
            timestamps,
            orientation_error_magnitude,
            label=f"{short_name}",
            linewidth=2,
        )

    # Configure position error plot
    axes[0].set_xlabel("Time [s]")
    axes[0].set_ylabel(r"Position Error $e_{\mathrm{pos}}$ [mm]")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # Configure orientation error plot
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel(r"Orientation Error $e_{\mathrm{rot}}$ [rad]")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()

    # Save as PDF
    filenames_str = "_vs_".join([d["filename"].replace(".npz", "") for d in datasets])
    pdf_filename = f"sporadic_pose_comparison_{filenames_str}.pdf"
    if len(pdf_filename) > 200:  # Avoid very long filenames
        pdf_filename = f"sporadic_pose_comparison_{len(datasets)}_experiments.pdf"

    plt.savefig("output/" + pdf_filename, bbox_inches="tight", dpi=300)
    print(f"Comparison figure saved as: {pdf_filename}")

    plt.show()


def main():
    """Main function to select and plot sporadic pose experiment data."""
    print("=== Sporadic Pose Error Plotter ===")

    # Get current directory NPZ files
    output_dir = os.getcwd() + "/output"
    if not os.path.exists(output_dir):
        print(f"Output directory does not exist: {output_dir}")
        print("Make sure to run sporadic pose experiments first.")
        return

    npz_files = list_sporadic_pose_files(output_dir)

    if not npz_files:
        print("No sporadic pose NPZ files found in output directory.")

        # Ask user for custom path
        custom_path = input("Enter path to NPZ file or directory: ").strip()
        if os.path.isfile(custom_path) and custom_path.endswith(".npz"):
            npz_files = [os.path.basename(custom_path)]
            output_dir = os.path.dirname(custom_path)
        elif os.path.isdir(custom_path):
            output_dir = custom_path
            npz_files = list_sporadic_pose_files(output_dir)

        if not npz_files:
            print("No sporadic pose NPZ files found. Exiting.")
            return

    print(f"\nFound {len(npz_files)} sporadic pose experiment file(s) in {output_dir}:")

    # List available files
    for i, filename in enumerate(npz_files, 1):
        filepath = os.path.join(output_dir, filename)
        file_size = os.path.getsize(filepath)
        mod_time = os.path.getmtime(filepath)
        mod_time_str = datetime.fromtimestamp(mod_time).strftime("%Y-%m-%d %H:%M:%S")
        print(f"  {i}. {filename} ({file_size / 1024:.1f} KB, modified: {mod_time_str})")

    # Ask for single or multiple file selection
    print("\nSelection options:")
    print("1. Plot single experiment")
    print("2. Plot multiple experiments (comparison)")

    selection_choice = input("Select option (1-2, default: 1): ").strip()

    if selection_choice == "2":
        # Multiple file selection
        print(
            "\nSelect multiple files to compare (enter numbers separated by commas, e.g., '1,2,3'):"
        )
        while True:
            try:
                choices = input(
                    f"Enter file numbers (1-{len(npz_files)}, or 'q' to quit): "
                ).strip()

                if choices.lower() == "q":
                    print("Exiting.")
                    return

                # Parse comma-separated indices
                file_indices = [int(x.strip()) - 1 for x in choices.split(",")]

                # Validate all indices
                if all(0 <= idx < len(npz_files) for idx in file_indices):
                    if len(file_indices) < 2:
                        print("Please select at least 2 files for comparison.")
                        continue
                    if len(file_indices) > 10:
                        print("Maximum 10 files allowed for comparison.")
                        continue
                    selected_files = [npz_files[idx] for idx in file_indices]
                    break
                else:
                    print(f"Please enter valid numbers between 1 and {len(npz_files)}")

            except ValueError:
                print("Please enter valid numbers separated by commas or 'q' to quit")

        # Load all selected files
        datasets = []
        for filename in selected_files:
            filepath = os.path.join(output_dir, filename)
            print(f"Loading: {filename}")

            try:
                data = load_and_validate_sporadic_pose_data(filepath)
                datasets.append({"data": data, "filename": filename})
                print(f"  Successfully loaded {len(data['timestamps'])} data points")
            except Exception as e:
                print(f"  Failed to load {filename}: {e}")

        if datasets:
            print(f"\nLoaded {len(datasets)} files for comparison")
            print("Creating comparison plot...")
            plot_comparison_sporadic_pose(datasets)
        else:
            print("No valid files loaded for comparison")

    else:
        # Single file selection
        while True:
            try:
                choice = input(
                    f"\nSelect file to plot (1-{len(npz_files)}, or 'q' to quit): "
                ).strip()

                if choice.lower() == "q":
                    print("Exiting.")
                    return

                file_index = int(choice) - 1
                if 0 <= file_index < len(npz_files):
                    selected_file = npz_files[file_index]
                    break
                else:
                    print(f"Please enter a number between 1 and {len(npz_files)}")

            except ValueError:
                print("Please enter a valid number or 'q' to quit")

        # Load and plot the selected file
        filepath = os.path.join(output_dir, selected_file)
        print(f"\nLoading: {filepath}")

        try:
            data = load_and_validate_sporadic_pose_data(filepath)
            print(f"Successfully loaded {len(data['timestamps'])} data points")
            plot_sporadic_pose_errors(data, selected_file)
        except Exception as e:
            print(f"Failed to load or validate file: {e}")


if __name__ == "__main__":
    main()

