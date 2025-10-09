#!/usr/bin/env python3
"""Script to check CRISP configuration paths and files."""

from crisp_py.camera.camera import list_camera_configs
from crisp_py.config.path import CRISP_CONFIG_PATHS
from crisp_py.gripper.gripper import list_gripper_configs
from crisp_py.robot import list_robot_configs

try:
    from rich import print
except ImportError:
    print = print

print("CRISP Configuration Files:")
print("-------------------------")
print(f"CRISP Config Paths: {', '.join([str(p) for p in CRISP_CONFIG_PATHS])}")
print("-------------------------")
print("Robot Configurations:")
print(list_robot_configs())
print("\nCamera Configurations:")
print(list_camera_configs())
print("\nGripper Configurations:")
print(list_gripper_configs())
print("-------------------------")

print(
    "Check: 'https://utiasdsl.github.io/crisp_controllers/misc/create_own_config/' for more details on how to create your own configuration files."
)
