"""Gripper configuration module."""

from dataclasses import dataclass
from pathlib import Path

import yaml

from crisp_py.config.path import find_config


@dataclass
class GripperConfig:
    """Gripper default config.

    Can be extented to be used with other grippers.

    Attributes:
        min_value (float): Minimum gripper value (fully closed).
        max_value (float): Maximum gripper value (fully open).
        command_topic (str): Topic to publish gripper commands to.
        joint_state_topic (str): Topic to subscribe for joint states.
        reboot_service (str): Service to reboot the gripper.
        enable_torque_service (str): Service to enable torque on the gripper.
        index (int): Index of the gripper joint in the joint states message.
        publish_frequency (float): Frequency to publish gripper state.
        max_joint_delay (float): Maximum delay for joint state updates.
        max_delta (float): Maximum change in gripper value per update.
        use_gripper_command_action (bool): Whether to use GripperCommandAction.
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
    use_gripper_command_action: bool = False
    max_effort: float = 10.0

    @classmethod
    def from_yaml(cls, path: str | Path, **overrides) -> "GripperConfig":  # noqa: ANN003
        """Create a GripperConfig from a YAML configuration file.

        Args:
            path (str | Path): Path to the YAML configuration file. Can be a filename
                               (searched in config paths) or a full path.
            **overrides: Additional parameters to override YAML values
        """
        if isinstance(path, str):
            found_path = find_config(path)
            if found_path is None:
                if Path(path).is_absolute() and Path(path).exists():
                    full_path = Path(path)
                else:
                    project_root_path = Path(__file__).parent.parent.parent
                    full_path = project_root_path / path
            else:
                full_path = found_path
        elif isinstance(path, Path):
            full_path = path
        else:
            raise TypeError("Path must be a string or a Path object.")

        with open(full_path, "r") as file:
            config = yaml.safe_load(file) or {}

            config_data = {
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

            config_data.update(overrides)

        return cls(**config_data)
