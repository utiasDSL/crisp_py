"""Sensor configuration classes for sensor objects."""

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(kw_only=True)
class SensorConfig:
    """Default sensor configuration."""

    shape: tuple[int, ...]
    sensor_type: str

    name: str = "sensor"
    data_topic: str = "sensor_data"
    max_data_delay: float = 1.0

    buffer_size: int | None = None

    @classmethod
    def from_yaml(cls, yaml_path: Path, **overrides) -> "SensorConfig":  # noqa: ANN003
        """Load config from YAML file with optional overrides.

        Args:
            yaml_path: Path to the YAML configuration file
            **overrides: Additional parameters to override YAML values

        Returns:
            SensorConfig: Configured sensor instance
        """
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}

        data.update(overrides)

        return cls(**data)
