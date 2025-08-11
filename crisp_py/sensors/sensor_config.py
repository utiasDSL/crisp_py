"""Sensor configuration classes for sensor objects."""

from dataclasses import dataclass
from typing import Literal


@dataclass(kw_only=True)
class SensorConfig:
    """Default sensor configuration."""

    shape: tuple[int, ...]
    sensor_type: Literal["empty", "float32", "force_torque"]

    name: str = "sensor"
    data_topic: str = "sensor_data"
    max_data_delay: float = 1.0


@dataclass
class EmptySensorConfig(SensorConfig):
    """Configuration for an empty sensor, used when using no sensor."""

    shape: tuple[int, ...] = (0,)
    sensor_type: Literal["empty", "float32", "force_torque"] = "empty"

    name: str = "empty_sensor"


@dataclass
class AnySkinSensorConfig(SensorConfig):
    """Configuration for the AnySkin sensor broadcasted by a node using the anyskin_ros2 wrapper: https://github.com/danielsanjosepro/anyskin_ros2."""

    shape: tuple[int, ...] = (5,)
    name: str = "anyskin_sensor"
    data_topic: str = "tactile_data"
    sensor_type: Literal["empty", "float32", "force_torque"] = "float32"


@dataclass
class ForceTorqueSensorConfig(SensorConfig):
    """Configuration for a force-torque (i.e. wrench) sensor."""

    shape: tuple[int, ...] = (6,)
    name: str = "ft_sensor"
    data_topic: str = "external_wrench"
    sensor_type: Literal["empty", "float32", "force_torque"] = "force_torque"


def make_sensor_config(
    sensor_type: Literal["empty", "float32", "force_torque"],
    **kwargs,  # noqa: ANN003
) -> SensorConfig:
    """Factory function to create a sensor configuration based on the type."""
    if sensor_type == "empty":
        return EmptySensorConfig(**kwargs)
    elif sensor_type == "float32":
        return AnySkinSensorConfig(**kwargs)
    elif sensor_type == "force_torque":
        return ForceTorqueSensorConfig(**kwargs)
    else:
        raise ValueError(f"Unknown sensor type: {sensor_type}")
