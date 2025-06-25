"""Initialize the camera module."""

from crisp_py.camera.camera import Camera
from crisp_py.camera.camera_config import CameraConfig, FrankaCameraConfig

__import__ = [Camera, CameraConfig, FrankaCameraConfig]
