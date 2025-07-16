"""Camera configuration class."""

from dataclasses import dataclass


@dataclass
class CameraConfig:
    """Default camera configuration."""

    camera_name: str = "camera"
    camera_frame: str = "camera_link"

    resolution: tuple[int, int] | None = None

    camera_color_image_topic: str | None = None
    camera_color_info_topic: str | None = None
    
    max_image_delay: float = 1.0


@dataclass
class FrankaCameraConfig(CameraConfig):
    """Example camera configuration used with Franka."""

    camera_name: str = "franka"
    camera_frame: str = "franka_link"

    resolution: tuple[int, int] | None = (256, 256)

    camera_color_image_topic: str | None = "franka/color/image_raw"
    camera_color_info_topic: str | None = "franka/color/camera_info"
