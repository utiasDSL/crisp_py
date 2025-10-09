"""Gripper configuration module."""

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass
class GripperConfig:
    """Gripper default config.

    Can be extented to be used with other grippers.
    """

    min_value: float
    max_value: float
    command_topic: str = "gripper_position_controller/commands"
    joint_state_topic: str = "joint_states"
    reboot_service: str = "reboot_gripper"
    enable_torque_service: str = "dynamixel_hardware_interface/set_dxl_torque"
    index: int = 0
    publish_frequency: float = 30.0
    max_joint_delay: float = 1.0
    max_delta: float = 0.1

    @classmethod
    def from_yaml(cls, path: str | Path) -> "GripperConfig":
        """Create a GripperConfig from a YAML configuration file.

        Args:
            path (str | Path): Path to the YAML configuration file from the project root or directly full path from the filesystem.
        """
        if isinstance(path, str):
            project_root_path = Path(__file__).parent.parent.parent
            full_path = project_root_path / path
        elif isinstance(path, Path):
            full_path = path
        else:
            raise TypeError("Path must be a string or a Path object.")

        with open(full_path, "r") as file:
            config = yaml.safe_load(file)
            config = {
                "min_value": config.get("min_value", 0.0),
                "max_value": config.get("max_value", 1.0),
                "command_topic": config.get(
                    "command_topic", "gripper_position_controller/commands"
                ),
                "joint_state_topic": config.get("joint_state_topic", "joint_states"),
                "reboot_service": config.get("reboot_service", "reboot_gripper"),
                "enable_torque_service": config.get(
                    "enable_torque_service", "dynamixel_hardware_interface/set_dxl_torque"
                ),
                "index": config.get("index", 0),
                "publish_frequency": config.get("publish_frequency", 30.0),
                "max_delta": config.get("max_delta", 0.1),
                "max_joint_delay": config.get("max_joint_delay", 1.0),
            }
        return cls(**config)
