#!/usr/bin/env python

"""Simple example to manually calibrate the gripper."""

# %%
from pathlib import Path

from datetime import datetime
import yaml

# %%
from crisp_py.gripper import Gripper

gripper = Gripper()
gripper.wait_until_ready()

# %%

print("--- Move the gripper to the open position ---")
input()
min_width = gripper.width
# min_width = 0.0

print("--- Now move the gripper to the close position ---")
input()
max_width = gripper.width
# max_width = 0.5

# %% Saving to a valid config file
this_file_path = Path(__file__).parent.parent
config_file = f"gripper_config_{datetime.now().strftime('%Y-%m-%d-%H-%M')}.yaml"

print(f"Saving config file to: {this_file_path / 'config' / config_file}")

with open(this_file_path / "config" / config_file, "w") as file:
    gripper_config = {
        "min_width": min_width,
        "max_width": max_width,
    }
    yaml.dump(gripper_config, file)
