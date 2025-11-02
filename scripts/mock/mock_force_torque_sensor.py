#!/usr/bin/env python3
"""Mock Force-Torque sensor that publishes WrenchStamped messages to a ROS2 topic."""

import random
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3, WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class MockForceTorqueSensorNode(Node):
    """Mock force-torque sensor node that publishes WrenchStamped messages."""

    def __init__(
        self,
        topic_name: str = "/sensor/force_torque",
        publish_rate: float = 100.0,
        force_noise_amplitude: float = 0.5,
        torque_noise_amplitude: float = 0.1,
        frame_id: str = "force_torque_sensor",
    ):
        """Initialize the mock force-torque sensor node.

        Args:
            topic_name: Name of the topic to publish to
            publish_rate: Publishing frequency in Hz
            force_noise_amplitude: Amplitude of random noise to add to forces (N)
            torque_noise_amplitude: Amplitude of random noise to add to torques (Nm)
            frame_id: Frame ID for the sensor
        """
        super().__init__("mock_force_torque_sensor")

        self.force_noise_amplitude = force_noise_amplitude
        self.torque_noise_amplitude = torque_noise_amplitude
        self.frame_id = frame_id

        # Create publisher
        self.publisher = self.create_publisher(WrenchStamped, topic_name, qos_profile_sensor_data)

        # Create timer for publishing
        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.publish_sensor_data)

        self.get_logger().info(
            f"Mock Force-Torque sensor publishing to '{topic_name}' at {publish_rate} Hz"
        )

    def publish_sensor_data(self):
        """Publish mock force-torque sensor data."""
        msg = WrenchStamped()

        # Set header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Generate mock force data with some pattern and noise
        t = time.time()

        # Simulate some periodic forces (e.g., from robot motion)
        base_fx = 2.0 * np.sin(t * 0.5) + random.uniform(
            -self.force_noise_amplitude, self.force_noise_amplitude
        )
        base_fy = 1.5 * np.cos(t * 0.3) + random.uniform(
            -self.force_noise_amplitude, self.force_noise_amplitude
        )
        base_fz = (
            -9.81
            + 0.5 * np.sin(t * 0.7)
            + random.uniform(-self.force_noise_amplitude, self.force_noise_amplitude)
        )  # Gravity + variation

        msg.wrench.force = Vector3(x=base_fx, y=base_fy, z=base_fz)

        # Generate mock torque data
        base_tx = 0.1 * np.sin(t * 0.8) + random.uniform(
            -self.torque_noise_amplitude, self.torque_noise_amplitude
        )
        base_ty = 0.15 * np.cos(t * 0.4) + random.uniform(
            -self.torque_noise_amplitude, self.torque_noise_amplitude
        )
        base_tz = 0.05 * np.sin(t * 1.2) + random.uniform(
            -self.torque_noise_amplitude, self.torque_noise_amplitude
        )

        msg.wrench.torque = Vector3(x=base_tx, y=base_ty, z=base_tz)

        self.publisher.publish(msg)


def main(args=None):  # noqa: ANN001
    """Main entry point."""
    rclpy.init(args=args)

    # Parse command line arguments or use defaults
    import argparse

    parser = argparse.ArgumentParser(description="Mock Force-Torque Sensor")
    parser.add_argument("--topic", default="/sensor/force_torque", help="Topic name")
    parser.add_argument("--rate", type=float, default=100.0, help="Publishing rate (Hz)")
    parser.add_argument("--force-noise", type=float, default=0.5, help="Force noise amplitude (N)")
    parser.add_argument(
        "--torque-noise", type=float, default=0.1, help="Torque noise amplitude (Nm)"
    )
    parser.add_argument("--frame-id", default="force_torque_sensor", help="Frame ID")

    parsed_args = parser.parse_args()

    node = MockForceTorqueSensorNode(
        topic_name=parsed_args.topic,
        publish_rate=parsed_args.rate,
        force_noise_amplitude=parsed_args.force_noise,
        torque_noise_amplitude=parsed_args.torque_noise,
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
