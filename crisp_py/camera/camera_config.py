"""Camera configuration class."""

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(kw_only=True)
class CameraConfig:
    """Default camera configuration."""

    camera_color_image_topic: str
    camera_color_info_topic: str
    resolution: tuple[int, int]

    camera_name: str = "camera"
    camera_frame: str = "camera_link"

    max_image_delay: float = 1.0

    resolution: tuple[int, int] | None = None
    crop_width: tuple[int, int] | None = None
    crop_height: tuple[int, int] | None = None

    @classmethod
    def from_yaml(cls, yaml_path: Path, **overrides) -> "CameraConfig":  # noqa: ANN003
        """Load config from YAML file with optional overrides.

        Args:
            yaml_path: Path to the YAML configuration file
            **overrides: Additional parameters to override YAML values

        Returns:
            CameraConfig: Configured camera instance
        """
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}

        # Apply overrides
        data.update(overrides)

        return cls(**data)


class DummyCameraConfig:
    """Dummy camera configuration class for testing purposes."""

    camera_color_image_topic: str = "dummy_camera/color/image_raw"
    camera_color_info_topic: str = "dummy_camera/color/camera_info"
    resolution: tuple[int, int] = (640, 480)
    camera_name: str = "dummy_camera"
    camera_frame: str = "dummy_camera_link"
    max_image_delay: float = 1.0
