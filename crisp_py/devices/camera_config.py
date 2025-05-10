"""Camera configuration class."""

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class CameraConfig:

    camera_name: str = "camera"
    camera_frame: str = "camera_link"

    resolution: Optional[Tuple[int, int]] = None

    @property
    def camera_color_image_topic(self) -> str:
        return f'{self.camera_name}/color/image_raw'

    @property
    def camera_color_info_topic(self) -> str:
        return f'{self.camera_name}/color/camera_info'