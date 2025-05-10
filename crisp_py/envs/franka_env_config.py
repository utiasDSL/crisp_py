from dataclasses import dataclass
from typing import List

from crisp_py.robot_config import FrankaConfig, RobotConfig
from crisp_py.devices.camera_config import CameraConfig
from crisp_py.gripper.gripper import GripperConfig


@dataclass
class FrankaEnvConfig:
    """Franka Gym Environment Configuration.

    This class is a configuration for the Franka Gym Environment.
    It contains the robot and camera configurations.
    """
    control_frequency: float = 10.0

    gripper_threshold: float = 0.1

    namespace: str = "left"

    robot_config: FrankaConfig = FrankaConfig()
    gripper_config: GripperConfig = GripperConfig()
    camera_configs: List[CameraConfig] = [
        CameraConfig(camera_name="primary", camera_frame="primary_camera_link"),
        CameraConfig(camera_name="wrist", camera_frame="wrist_camera_link"),
    ]