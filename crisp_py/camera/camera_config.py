"""Camera configuration class."""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class CameraConfig:

    camera_name: str = "camera"
    camera_frame: str = "camera_link"

    resolution: Optional[Tuple[int, int]] = None

    camera_color_image_topic: str = None

    camera_color_info_topic: str = None

@dataclass
class PrimaryCameraConfig:

    camera_name: str = "primary"
    camera_frame: str = "primary_link"

    resolution: Optional[Tuple[int, int]] = (256, 256)

    camera_color_image_topic: str = "right_third_person_camera/color/image_raw"
    camera_color_info_topic: str = "right_third_person_camera/color/camera_info"

@dataclass
class WristCameraConfig:

    camera_name: str = "wrist"
    camera_frame: str = "wrist_link"

    resolution: Optional[Tuple[int, int]] = (256, 256)

    camera_color_image_topic: str = "right_wrist_camera/color/image_rect_raw"
    camera_color_info_topic: str = "right_wrist_camera/color/camera_info"