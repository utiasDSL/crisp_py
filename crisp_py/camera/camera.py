"""Class defining a camera object."""

import threading
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
import rclpy.executors
import yaml
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from crisp_py.camera.camera_config import CameraConfig, DummyCameraConfig
from crisp_py.config.path import find_config, list_configs_in_folder
from crisp_py.utils.callback_monitor import CallbackMonitor


class Camera:
    """High level interface for managing cameras in ROS2.

    Images are stored in RGB8 format as a numpy array of shape (H, W, 3).
    If resolution is set in the camera configuration, the image will be resized while maintaining the aspect ratio.
    (Note: The image will be cropped to fit the target resolution if necessary.)
    """

    THREADS_REQUIRED = 2

    def __init__(
        self,
        node: Node | None = None,
        namespace: str = "",
        config: CameraConfig | None = None,
        spin_node: bool = True,
    ):
        """Initialize the camera.

        Args:
            node (Node, optional): ROS2 node to use. If None, creates a new node.
            namespace (str, optional): ROS2 namespace for the camera.
            config (CameraConfig): Camera configuration.
            spin_node (bool, optional): Whether to spin the node in a separate thread.
        """
        self.config = (
            config if config else DummyCameraConfig()
        )  # NOTE: it would be better to no allow None here

        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node(self.config.camera_name, namespace=namespace)
        else:
            self.node = node

        self._current_image: Optional[np.ndarray] = None
        self._namespace = namespace
        self._callback_monitor = CallbackMonitor(
            self.node,
            stale_threshold=self.config.max_image_delay,
        )

        self.cv_bridge = CvBridge()

        self._camera_subscriber = self.node.create_subscription(
            CompressedImage,
            f"{self.config.camera_color_image_topic}/compressed",
            self._callback_monitor.monitor(
                f"{self._namespace.capitalize()} Camera {self.config.camera_name} Image".strip(),
                self._callback_current_color_image,
            ),
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )
        assert (
            self.config.camera_color_info_topic is not None or self.config.resolution is not None
        ), "You have to set resolution or camera info topic"
        if self.config.camera_color_info_topic is None or self.config.resolution is None:
            print(
                "[Camera warning] You have set resolution and camera info topic, camera info topic will be ignored"
            )
        if self.config.camera_color_info_topic is not None:
            self.node.create_subscription(
                CameraInfo,
                self.config.camera_color_info_topic,
                self._callback_monitor.monitor(
                    f"{self._namespace.capitalize()} Camera {self.config.camera_name} Info".strip(),
                    self._callback_current_color_info,
                ),
                qos_profile_system_default,
                callback_group=ReentrantCallbackGroup(),
            )

        self._image_has_changed = False

        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

    @classmethod
    def from_yaml(
        cls,
        config_name: str,
        node: Node | None = None,
        namespace: str = "",
        spin_node: bool = True,
        **overrides,  # noqa: ANN003
    ) -> "Camera":
        """Create a Camera instance from a YAML configuration file.

        Args:
            config_name: Name of the config file (with or without .yaml extension)
            node: ROS2 node to use. If None, creates a new node.
            namespace: ROS2 namespace for the camera.
            spin_node: Whether to spin the node in a separate thread.
            **overrides: Additional parameters to override YAML values

        Returns:
            Camera: Configured camera instance

        Raises:
            FileNotFoundError: If the config file is not found
        """
        if not config_name.endswith(".yaml"):
            config_name += ".yaml"

        config_path = find_config(f"cameras/{config_name}")
        if config_path is None:
            config_path = find_config(config_name)

        if config_path is None:
            raise FileNotFoundError(
                f"Camera config file '{config_name}' not found in any CRISP config paths"
            )

        with open(config_path, "r") as f:
            data = yaml.safe_load(f) or {}

        data.update(overrides)

        namespace = data.pop("namespace", namespace)
        config_data = data.pop("camera_config", data)

        config = CameraConfig(**config_data)

        return cls(
            node=node,
            namespace=namespace,
            config=config,
            spin_node=spin_node,
        )

    @staticmethod
    def list_configs() -> list[str]:
        """List all available camera configurations."""
        configs = list_configs_in_folder("cameras")
        return [config.stem for config in configs if config.suffix == ".yaml"]

    def _spin_node(self):
        if not rclpy.ok():
            rclpy.init()
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=self.THREADS_REQUIRED)
        executor.add_node(self.node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    def _uncompress(self, compressed_image: CompressedImage) -> Image:
        """Uncompress a CompressedImage message to an Image message."""
        return np.asarray(
            self.cv_bridge.compressed_imgmsg_to_cv2(compressed_image, desired_encoding="rgb8")
        )

    def has_image_changed_since_last_retrieval(self) -> bool:
        """Return true if the image has changed since the last time that the current_image has been accessed.

        This is useful to avoid processing the same image multiple times.

        Returns:
            bool: True if the image has changed since the last retrieval.
        """
        return self._image_has_changed

    @property
    def current_image(self) -> np.ndarray:
        """Get the current color image."""
        if self._current_image is None:
            raise RuntimeError(
                f"We have not received any images of camera {self.config.camera_name}. Call wait_until_ready to be sure that the camera is available!."
            )
        try:
            for callback_name in self._callback_monitor.callbacks.keys():
                if (
                    "Camera" in callback_name
                    and self.config.camera_name in callback_name
                    and "Image" in callback_name
                ):
                    image_callback_data = self._callback_monitor.get_callback_data(callback_name)
                    if image_callback_data and image_callback_data.is_stale:
                        self.node.get_logger().warn(
                            f"Camera {self.config.camera_name} image data is stale"
                        )
                    break
        except ValueError:
            pass
        self._image_has_changed = False
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
                error_msg = (
                    f"Timeout waiting for camera ({self.config.camera_name}) to become ready."
                )
                error_msg += (
                    f"Is the camera publishing to the topic {self._camera_subscriber.topic_name}?"
                )
                raise TimeoutError(error_msg)

    def _callback_current_color_image(self, msg: CompressedImage):
        """Receive and store the current image."""
        self._image_has_changed = True
        self._current_image = self.ros_msg_to_image(msg)

    def _callback_current_color_info(self, msg: CameraInfo):
        """Receive and store the current camera info."""
        if self.config.resolution is None:
            self.config.resolution = (msg.height, msg.width)

    def _image_to_array(self, msg: Image) -> np.ndarray:
        """Converts an Image message to a numpy array."""
        return np.asarray(self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8"))

    def _resize_with_aspect_ratio(
        self,
        image: np.ndarray,
        target_res: tuple,
        crop_height: tuple[int | float, int | float] | None = None,
        crop_width: tuple[int | float, int | float] | None = None,
    ) -> np.ndarray:
        """Resize an image to fit within a target resolution while maintaining aspect ratio, cropping if necessary."""
        image = self._pre_crop(image, crop_height, crop_width)

        h, w = image.shape[:2]
        target_h, target_w = target_res

        if h == target_h and w == target_w:
            return image

        scale = max(target_w / w, target_h / h)
        new_w, new_h = int(w * scale), int(h * scale)

        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)

        start_x = (new_w - target_w) // 2
        start_y = (new_h - target_h) // 2

        cropped_image = resized[start_y : start_y + target_h, start_x : start_x + target_w]

        return cropped_image

    def ros_msg_to_image(self, msg: CompressedImage) -> np.ndarray:
        """Convert a ROS message to numpy array for this camera configuration."""
        return self._resize_with_aspect_ratio(
            self._uncompress(msg),
            target_res=self.config.resolution,
            crop_width=self.config.crop_width,
            crop_height=self.config.crop_height,
        )

    def _pre_crop(
        self,
        image: np.ndarray,
        crop_height: tuple[int | float, int | float] | None,
        crop_width: tuple[int | float, int | float] | None,
    ) -> np.ndarray:
        """Crop the image according to specified height and width ranges.

        Args:
            image: Input image as a numpy array
            crop_height: Tuple specifying the height crop (start, end).
                        Use integers for absolute pixel values, or floats (0.0-1.0) for relative cropping.
            crop_width: Tuple specifying the width crop (start, end).
                       Use integers for absolute pixel values, or floats (0.0-1.0) for relative cropping.

        Returns:
            Cropped image as a numpy array

        Raises:
            ValueError: If crop parameters are invalid

        """
        if crop_height is not None:
            h = image.shape[0]
            if not isinstance(crop_height, (list, tuple)) or len(crop_height) != 2:
                raise ValueError(
                    f"Invalid crop_height {crop_height}: must be a list or tuple of length 2"
                )

            if isinstance(crop_height[0], float) or isinstance(crop_height[1], float):
                if not all(isinstance(x, float) for x in crop_height):
                    raise ValueError(
                        "crop_height values must be either all int or all float, not mixed."
                    )
                if not all(0.0 <= x <= 1.0 for x in crop_height):
                    raise ValueError(
                        f"Float crop_height values must be between 0.0 and 1.0. Got: {crop_height}"
                    )
                if crop_height[0] >= crop_height[1]:
                    raise ValueError(f"crop_height start must be less than end. Got: {crop_height}")

                crop_start = int(crop_height[0] * h)
                crop_end = int(crop_height[1] * h)
            else:
                if not all(isinstance(x, int) for x in crop_height):
                    raise ValueError(
                        "crop_height values must be either all int or all float, not mixed."
                    )
                if not (0 <= crop_height[0] < crop_height[1] <= h):
                    raise ValueError(
                        f"Invalid crop_height {crop_height}: must satisfy 0 <= start < end <= image height ({h})"
                    )
                crop_start = crop_height[0]
                crop_end = crop_height[1]

            image = image[crop_start:crop_end]

        if crop_width is not None:
            w = image.shape[1]
            if not isinstance(crop_width, (list, tuple)) or len(crop_width) != 2:
                raise ValueError(
                    f"Invalid crop_width {crop_width}: must be a list or tuple of length 2"
                )

            if isinstance(crop_width[0], float) or isinstance(crop_width[1], float):
                if not all(isinstance(x, float) for x in crop_width):
                    raise ValueError(
                        "crop_width values must be either all int or all float, not mixed."
                    )
                if not all(0.0 <= x <= 1.0 for x in crop_width):
                    raise ValueError(
                        f"Float crop_width values must be between 0.0 and 1.0. Got: {crop_width}"
                    )
                if crop_width[0] >= crop_width[1]:
                    raise ValueError(f"crop_width start must be less than end. Got: {crop_width}")

                crop_start = int(crop_width[0] * w)
                crop_end = int(crop_width[1] * w)
            else:
                if not all(isinstance(x, int) for x in crop_width):
                    raise ValueError(
                        "crop_width values must be either all int or all float, not mixed."
                    )
                if not (0 <= crop_width[0] < crop_width[1] <= w):
                    raise ValueError(
                        f"Invalid crop_width {crop_width}: must satisfy 0 <= start < end <= image width ({w})"
                    )
                crop_start = crop_width[0]
                crop_end = crop_width[1]

            image = image[:, crop_start:crop_end]

        return image


def make_camera(
    config_name: str | None = None,
    camera_config: Optional[CameraConfig] = None,
    node: "Node | None" = None,
    namespace: str = "",
    spin_node: bool = True,
    **overrides,  # noqa: ANN003
) -> Camera:
    """Factory function to create a Camera from a configuration file.

    Args:
        config_name: Name of the camera config file
        camera_config: CameraConfig instance to use.
        node: ROS2 node to use. If None, creates a new node.
        namespace: ROS2 namespace for the camera.
        spin_node: Whether to spin the node in a separate thread.
        **overrides: Additional parameters to override config values

    Returns:
        Camera: Configured camera instance

    Raises:
        FileNotFoundError: If the config file is not found
    """
    assert (not config_name and camera_config) or (config_name and not camera_config), (
        "Either config_name or camera_config must be provided, but not both."
    )

    if config_name is not None:
        return Camera.from_yaml(
            config_name=config_name,
            node=node,
            namespace=namespace,
            spin_node=spin_node,
            **overrides,
        )

    if overrides:
        for key, value in overrides.items():
            if hasattr(camera_config, key):
                setattr(camera_config, key, value)
            else:
                raise ValueError(f"Invalid override key: {key}")

    return Camera(
        node=node,
        namespace=namespace,
        config=camera_config,
        spin_node=spin_node,
    )


def list_camera_configs() -> list[str]:
    """List all available camera configurations."""
    return Camera.list_configs()
