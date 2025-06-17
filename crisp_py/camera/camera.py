"""Class defining a camera object."""

import cv2
import numpy as np
import threading
from typing import Optional, Tuple
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import CameraInfo, Image

from cv_bridge import CvBridge

from crisp_py.camera.camera_config import CameraConfig


class Camera:
    """High level interface for managing cameras in ROS2.

    Images are stored in RGB8 format as a numpy array of shape (H, W, 3).
    If resolution is set in the camera configuration, the image will be resized while maintaining the aspect ratio. 
    (Note: The image will be cropped to fit the target resolution if necessary.)
    """
    THREADS_REQUIRED = 2

    def __init__(self, node: Optional[Node] = None, namespace: str = "", config: Optional[CameraConfig] = None, spin_node: bool = True):
        """Initialize the camera.

        Args:
            node (Node, optional): ROS2 node to use. If None, creates a new node.
            namespace (str, optional): ROS2 namespace for the camera.
            config (CameraConfig): Camera configuration.
            spin_node (bool, optional): Whether to spin the node in a separate thread.
        """
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node(config.camera_name, namespace=namespace)
        else:
            self.node = node

        self.config = config if config else CameraConfig()

        self._current_image: Optional[np.ndarray] = None

        self.cv_bridge = CvBridge()

        self.node.create_subscription(
            Image,
            self.config.camera_color_image_topic,
            self._callback_current_color_image,
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )
        self.node.create_subscription(
            CameraInfo,
            self.config.camera_color_info_topic,
            self._callback_current_color_info,
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )

        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

    def _spin_node(self):
        if not rclpy.ok():
            rclpy.init()
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=self.THREADS_REQUIRED)
        executor.add_node(self.node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    @property
    def current_image(self) -> np.ndarray:
        """Get the current color image."""
        return self._current_image
    
    @property
    def resolution(self) -> Tuple[int, int]:
        """Get the current camera info."""
        return self.config.resolution if self.config.resolution is not None else (0, 0)
    
    def is_ready(self) -> bool:
        """Returns True if camera image and resolution are available."""
        return self._current_image is not None and self.config.resolution is not None
    
    def wait_until_ready(self, timeout: float = 10.0, check_frequency: float = 10.0):
        """Wait until camera image and resolution are available."""
        rate = self.node.create_rate(check_frequency)
        while not self.is_ready():
            rate.sleep()
            timeout -= 1.0 / check_frequency
            if timeout <= 0:
                raise TimeoutError(f"Timeout waiting for camera ({self.config.camera_name}) to become ready.")

    def _callback_current_color_image(self, msg: Image):
        """Receive and store the current image."""
        raw_image = self._image_to_array(msg)
        if self.config.resolution is not None:
            raw_image = self._resize_with_aspect_ratio(raw_image, self.config.resolution)

        self._current_image = raw_image

    def _callback_current_color_info(self, msg: CameraInfo):
        """Receive and store the current camera info."""
        if self.config.resolution is None:
            self.config.resolution = (msg.height, msg.width)

    def _image_to_array(self, msg: Image) -> np.ndarray:
        """Converts an Image message to a numpy array."""
        return np.asarray(self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8'))
    
    def _resize_with_aspect_ratio(self, image: np.ndarray, target_res: tuple):
        """
        Resize an image to fit within a target resolution while maintaining aspect ratio,
        cropping if necessary.
        """
        h, w = image.shape[:2]
        target_h, target_w = target_res
        
        scale = max(target_w / w, target_h / h)
        new_w, new_h = int(w * scale), int(h * scale)
        
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
        
        start_x = (new_w - target_w) // 2
        start_y = (new_h - target_h) // 2
        
        cropped_image = resized[start_y:start_y + target_h, start_x:start_x + target_w]
        
        return cropped_image
