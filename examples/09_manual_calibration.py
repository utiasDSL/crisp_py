#!/usr/bin/env python

"""Simple example to manually calibrate the gripper."""

# %%
from datetime import datetime
from pathlib import Path

import yaml

# %%
from crisp_py.gripper import Gripper

gripper = Gripper(namespace="follower")
gripper.wait_until_ready()

# %%
gripper.disable_torque()

# %%

print("--- Move the gripper to the closed position, then press enter. ---")
input()
min_value = gripper.raw_value

print("--- Now move the gripper to the open position, then press enter. ---")
input()
max_value = gripper.raw_value

# %%
print(f"Measured min_value is {min_value} and max_value is {max_value}")

# %% Saving to a valid config file
this_file_path = Path(__file__).parent.parent
config_file = f"gripper_config_{datetime.now().strftime('%Y-%m-%d-%H-%M')}.yaml"

print(f"Saving config file to: {this_file_path / 'config' / config_file}")

with open(this_file_path / "config" / config_file, "w") as file:
    gripper_config = {"min_value": min_value, "max_value": max_value}
    yaml.dump(gripper_config, file)

gripper.shutdown()
