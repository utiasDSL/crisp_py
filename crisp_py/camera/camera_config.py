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
class FrankaCameraConfig:

    camera_name: str = "franka"
    camera_frame: str = "franka_link"

    resolution: Optional[Tuple[int, int]] = (256, 256)

    camera_color_image_topic: str = "franka/color/image_raw"
    camera_color_info_topic: str = "franka/color/camera_info"