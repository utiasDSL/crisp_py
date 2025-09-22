"""Type definitions for CRISP components using Literal and Union types."""

from typing import Literal, Union

from crisp_py.constants import (
    CAMERA_TYPES,
    COMPONENT_TYPES,
    CONFIGURATION_SOURCES,
    GRIPPER_TYPES,
    ROBOT_TYPES,
    SENSOR_TYPES,
)

ComponentType = Literal[COMPONENT_TYPES]
RobotType = Literal[ROBOT_TYPES]
CameraType = Literal[CAMERA_TYPES]
GripperType = Literal[GRIPPER_TYPES]
SensorType = Literal[SENSOR_TYPES]
ConfigurationSource = Literal[CONFIGURATION_SOURCES]

CustomRobotType = Union[RobotType, str]
CustomCameraType = Union[CameraType, str]
CustomGripperType = Union[GripperType, str]
CustomSensorType = Union[SensorType, str]
