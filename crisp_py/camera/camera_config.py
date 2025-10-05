"""Camera configuration class."""

from dataclasses import dataclass


@dataclass(kw_only=True)
class CameraConfig:
    """Default camera configuration."""

    camera_color_image_topic: str
    camera_color_info_topic: str
    resolution: tuple[int, int]

    camera_name: str = "camera"
    camera_frame: str = "camera_link"

    max_image_delay: float = 1.0


class DummyCameraConfig:
    """Dummy camera configuration class for testing purposes."""

    camera_color_image_topic: str = "dummy_camera/color/image_raw"
    camera_color_info_topic: str = "dummy_camera/color/camera_info"
    resolution: tuple[int, int] = (640, 480)
    camera_name: str = "dummy_camera"
    camera_frame: str = "dummy_camera_link"
    max_image_delay: float = 1.0
