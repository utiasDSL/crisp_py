"""Sensor configuration classes for sensor objects."""

from dataclasses import dataclass


@dataclass
class SensorConfig:
    """Default sensor configuration."""

    shape: tuple[int, ...]

    name: str = "sensor"
    data_topic: str = "sensor_data"
    max_data_delay: float = 1.0


@dataclass
class AnySkinSensorConfig(SensorConfig):
    """Configuration for the AnySkin sensor broadcasted by a node using the anyskin_ros2 wrapper: https://github.com/danielsanjosepro/anyskin_ros2."""

    shape: tuple[int, ...] = (5,)
    name: str = "anyskin_sensor"
    data_topic: str = "tactile_data"
