#!/usr/bin/env python

"""Simple example to manually calibrate the gripper."""

# %%
import argparse
from pathlib import Path

try:
    from rich import print
except ImportError:
    print("Please install rich to see colored output. Not required for functionality.")

import yaml

# %%
from crisp_py.gripper import Gripper, GripperConfig

# Parse command line arguments
parser = argparse.ArgumentParser(description="Manually calibrate the gripper.")
parser.add_argument(
    "--config-file",
    type=str,
    default="gripper.yaml",
    help="Name of the output config file (default: gripper_right.yaml)",
)
parser.add_argument(
    "--namespace", type=str, default="gripper", help="Robot gripper namespace (default: gripper)"
)
parser.add_argument(
    "--joint-state-topic",
    type=str,
    default="gripper_state_broadcaster/joint_states",
    help="Joint state topic (default: gripper_state_broadcaster/joint_states)",
)
parser.add_argument(
    "--index", type=int, default=0, help="Index of the gripper to calibrate (default: 0)"
)
args = parser.parse_args()

print("\n[bold green]Starting manual calibration of the gripper :hand: [/bold green]")
print(f"\nSaving final config to: [bold blue]{args.config_file}[/bold blue]")
print(f"Using namespace: [bold blue]{args.namespace}[/bold blue]")
print(f"Using joint state topic: [bold blue]{args.joint_state_topic}[/bold blue]")
print(f"Using gripper index: [bold blue]{args.index}[/bold blue]")
print(":bulb: To modifiy these parameters, add the [bold]--help[/bold] flag to this command.\n")


gripper_config = GripperConfig(0.0, 1.0, joint_state_topic=args.joint_state_topic, index=args.index)
gripper = Gripper(gripper_config=gripper_config, namespace=args.namespace)
gripper.wait_until_ready()

# %%
gripper.disable_torque()

# %%

print(":right_arrow: Move the gripper to the closed position, then press enter.")
input()
min_value = gripper.raw_value
print(f":white_check_mark: Measured min_value is [b]{min_value}[/b].\n")
print(":right_arrow: Now move the gripper to the open position, then press enter.")
input()
max_value = gripper.raw_value
print(f":white_check_mark: Measured max_value is [b]{max_value}[/b].\n")

print(
    f":hourglass_flowing_sand: Press enter to save the config file with min_value=[b]{min_value}[/b] and max_value=[b]{max_value}[/b]."
)
input()

# %% Saving to a valid config file
this_file_path = Path(__name__).parent.parent
config_file = args.config_file

print(f":right_arrow: Saving config file to: {this_file_path / 'config' / config_file}")

with open(this_file_path / "config" / config_file, "w") as file:
    gripper_config = {
        "min_value": min_value,
        "max_value": max_value,
        "joint_state_topic": args.joint_state_topic,
    }
    yaml.dump(gripper_config, file)

print(
    f":white_check_mark: Config file saved successfully to [b]{this_file_path / 'config' / config_file}[/b]."
)
print("[bold green]Manual calibration completed successfully! :hand:[/bold green]\n")

gripper.shutdown()
