#!/usr/bin/env python3
"""Mock Float32 array sensor that publishes Float32MultiArray messages to a ROS2 topic."""

import random
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray


class MockFloat32ArraySensorNode(Node):
    """Mock sensor node that publishes Float32MultiArray messages."""

    def __init__(
        self,
        topic_name: str = "/sensor/float32_array",
        publish_rate: float = 10.0,
        array_size: int = 6,
        noise_amplitude: float = 0.1,
    ):
        """Initialize the mock sensor node.

        Args:
            topic_name: Name of the topic to publish to
            publish_rate: Publishing frequency in Hz
            array_size: Size of the float32 array
            noise_amplitude: Amplitude of random noise to add
        """
        super().__init__("mock_float32_array_sensor")

        self.array_size = array_size
        self.noise_amplitude = noise_amplitude

        self.publisher = self.create_publisher(
            Float32MultiArray, topic_name, qos_profile_sensor_data
        )

        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.publish_sensor_data)

        self.get_logger().info(
            f"Mock Float32Array sensor publishing to '{topic_name}' at {publish_rate} Hz"
        )

    def publish_sensor_data(self):
        """Publish mock sensor data."""
        msg = Float32MultiArray()

        t = time.time()
        base_values = [
            np.sin(t * 0.5) + random.uniform(-self.noise_amplitude, self.noise_amplitude),
            np.cos(t * 0.3) + random.uniform(-self.noise_amplitude, self.noise_amplitude),
            np.sin(t * 0.7) * 0.5 + random.uniform(-self.noise_amplitude, self.noise_amplitude),
        ]

        while len(base_values) < self.array_size:
            base_values.append(random.uniform(-self.noise_amplitude, self.noise_amplitude))

        msg.data = base_values[: self.array_size]

        self.publisher.publish(msg)


def main(args=None):  # noqa: ANN001
    """Main entry point."""
    rclpy.init(args=args)

    import argparse

    parser = argparse.ArgumentParser(description="Mock Float32Array Sensor")
    parser.add_argument("--topic", default="/sensor/float32_array", help="Topic name")
    parser.add_argument("--rate", type=float, default=10.0, help="Publishing rate (Hz)")
    parser.add_argument("--size", type=int, default=6, help="Array size")
    parser.add_argument("--noise", type=float, default=0.1, help="Noise amplitude")

    parsed_args = parser.parse_args()

    node = MockFloat32ArraySensorNode(
        topic_name=parsed_args.topic,
        publish_rate=parsed_args.rate,
        array_size=parsed_args.size,
        noise_amplitude=parsed_args.noise,
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
