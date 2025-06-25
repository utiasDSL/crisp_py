"""Camera configuration class."""

from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class CameraConfig:
    """Default camera configuration."""

    camera_name: str = "camera"
    camera_frame: str = "camera_link"

    resolution: Optional[Tuple[int, int]] = None

    camera_color_image_topic: Optional[str] = None

    camera_color_info_topic: Optional[str] = None


@dataclass
class FrankaCameraConfig:
    """Example camera configuration used with Franka."""

    camera_name: str = "franka"
    camera_frame: str = "franka_link"

    resolution: Optional[Tuple[int, int]] = (256, 256)

    camera_color_image_topic: str = "franka/color/image_raw"
    camera_color_info_topic: str = "franka/color/camera_info"
