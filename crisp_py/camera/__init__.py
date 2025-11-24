"""Initialize the camera module."""

from crisp_py.camera.camera import Camera, make_camera
from crisp_py.camera.camera_config import CameraConfig

__import__ = [Camera, CameraConfig, make_camera]
