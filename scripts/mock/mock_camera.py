#!/usr/bin/env python3
"""Mock Camera that publishes compressed images and camera info to ROS2 topics."""

import argparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage


class MockCameraNode(Node):
    """Mock camera node that publishes compressed images and camera info."""

    def __init__(
        self,
        image_topic: str = "/camera/color/image_raw",
        info_topic: str = "/camera/color/camera_info",
        publish_rate: float = 30.0,
        width: int = 256,
        height: int = 256,
        frame_id: str = "camera_link",
    ):
        """Initialize the mock camera node.

        Args:
            image_topic: Topic name for publishing images
            info_topic: Topic name for publishing camera info
            publish_rate: Publishing frequency in Hz
            width: Image width in pixels
            height: Image height in pixels
            frame_id: Frame ID for camera
        """
        super().__init__("mock_camera")

        self.width = width
        self.height = height
        self.frame_id = frame_id
        self.cv_bridge = CvBridge()

        self.image_publisher = self.create_publisher(
            CompressedImage, f"{image_topic}/compressed", qos_profile_sensor_data
        )
        self.info_publisher = self.create_publisher(CameraInfo, info_topic, qos_profile_sensor_data)

        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.publish_data)

        self.camera_info = self._create_camera_info()

        self.get_logger().info(
            f"Mock Camera publishing to '{image_topic}' and '{info_topic}' at {publish_rate} Hz"
        )

    def _create_camera_info(self) -> CameraInfo:
        """Create a camera info message with reasonable defaults."""
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height

        fx = fy = self.width * 0.8
        cx = self.width / 2.0
        cy = self.height / 2.0

        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]

        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        return info

    def _generate_test_image(self) -> np.ndarray:
        """Generate a test image with moving patterns."""
        return np.random.randint(0, 256, size=(self.height, self.width, 3), dtype=np.uint8)

    def publish_data(self):
        """Publish camera image and info."""
        image = self._generate_test_image()

        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_msg.header.frame_id = self.frame_id
        compressed_msg.format = "jpeg"

        success, encoded_image = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if success:
            compressed_msg.data = encoded_image.tobytes()
        else:
            self.get_logger().error("Failed to encode image as JPEG")
            return

        self.camera_info.header.stamp = compressed_msg.header.stamp

        self.image_publisher.publish(compressed_msg)
        self.info_publisher.publish(self.camera_info)


def main(args=None):  # noqa: ANN001
    """Main entry point."""
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="Mock Camera")
    parser.add_argument("--image-topic", default="/camera/color/image_raw", help="Image topic name")
    parser.add_argument("--info-topic", default="/camera/color/camera_info", help="Info topic name")
    parser.add_argument("--rate", type=float, default=30.0, help="Publishing rate (Hz)")
    parser.add_argument("--width", type=int, default=256, help="Image width")
    parser.add_argument("--height", type=int, default=256, help="Image height")
    parser.add_argument("--frame-id", default="camera_link", help="Frame ID")

    parsed_args = parser.parse_args()

    node = MockCameraNode(
        image_topic=parsed_args.image_topic,
        info_topic=parsed_args.info_topic,
        publish_rate=parsed_args.rate,
        width=parsed_args.width,
        height=parsed_args.height,
        frame_id=parsed_args.frame_id,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
