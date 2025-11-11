#!/usr/bin/env python3
"""Mock Gripper that publishes joint states and responds to commands on ROS2 topics."""

import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger


class MockGripperNode(Node):
    """Mock gripper node that simulates gripper behavior."""

    def __init__(
        self,
        command_topic: str = "gripper_position_controller/commands",
        joint_state_topic: str = "joint_states",
        reboot_service: str = "reboot_gripper",
        enable_torque_service: str = "dynamixel_hardware_interface/set_dxl_torque",
        publish_rate: float = 30.0,
        min_value: float = 0.0,
        max_value: float = 0.08,
        joint_name: str = "gripper_joint",
        namespace: str = "",
        index: int = 0,
    ):
        """Initialize the mock gripper node.

        Args:
            command_topic: Topic name for receiving gripper commands
            joint_state_topic: Topic name for publishing joint states
            reboot_service: Service name for rebooting gripper
            enable_torque_service: Service name for enabling/disabling torque
            publish_rate: Publishing frequency in Hz
            min_value: Minimum gripper value (closed)
            max_value: Maximum gripper value (open)
            joint_name: Name of the gripper joint
            namespace: Gripper namespace
            index: Index of gripper joint in joint state message
        """
        super().__init__("mock_gripper")

        self.min_value = min_value
        self.max_value = max_value
        self.joint_name = joint_name
        self.namespace = namespace
        self.index = index

        self.current_position = min_value  # Start closed
        self.target_position = min_value
        self.current_velocity = 0.0
        self.current_effort = 0.0
        self.torque_enabled = True

        self.max_velocity = 0.02  # m/s for position control
        self.position_tolerance = 0.001  # m

        self.joint_publisher = self.create_publisher(
            JointState, joint_state_topic, qos_profile_system_default
        )

        self.command_subscriber = self.create_subscription(
            Float64MultiArray, command_topic, self.command_callback, qos_profile_system_default
        )

        self.reboot_service = self.create_service(Trigger, reboot_service, self.reboot_callback)
        self.enable_torque_service = self.create_service(
            SetBool, enable_torque_service, self.enable_torque_callback
        )

        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.publish_joint_state)

        self.get_logger().info(
            f"Mock Gripper publishing to '{joint_state_topic}' at {publish_rate} Hz"
        )
        self.get_logger().info(f"Range: [{min_value:.3f}, {max_value:.3f}]")

    def command_callback(self, msg: Float64MultiArray):
        """Handle incoming gripper position commands."""
        if not self.torque_enabled:
            self.get_logger().warn("Gripper torque is disabled, ignoring command")
            return

        if len(msg.data) > self.index:
            target = msg.data[self.index]
            self.target_position = np.clip(target, self.min_value, self.max_value)
            self.get_logger().debug(f"New target position: {self.target_position:.3f}")

    def reboot_callback(self, request, response):  # noqa: ANN001, ANN201
        """Handle gripper reboot service calls."""
        self.get_logger().info("Rebooting gripper...")

        self.current_position = self.min_value
        self.target_position = self.min_value
        self.current_velocity = 0.0
        self.current_effort = 0.0
        self.torque_enabled = True

        response.success = True
        response.message = "Gripper rebooted successfully"
        return response

    def enable_torque_callback(self, request, response):  # noqa: ANN001, ANN201
        """Handle enable/disable torque service calls."""
        self.torque_enabled = request.data
        status = "enabled" if self.torque_enabled else "disabled"
        self.get_logger().info(f"Gripper torque {status}")

        if not self.torque_enabled:
            self.current_effort = 0.0

        response.success = True
        response.message = f"Gripper torque {status}"
        return response

    def update_gripper_state(self):
        """Update gripper position based on target."""
        if not self.torque_enabled:
            self.current_velocity = 0.0
            self.current_effort = 0.0
            return

        error = self.target_position - self.current_position

        if abs(error) < self.position_tolerance:
            self.current_velocity = 0.0
            self.current_effort = 5.0
            return

        direction = 1.0 if error > 0 else -1.0
        self.current_velocity = direction * min(self.max_velocity, abs(error) * 10.0)

        dt = 1.0 / 30.0  # Assume 30 Hz update rate
        self.current_position += self.current_velocity * dt
        self.current_position = np.clip(self.current_position, self.min_value, self.max_value)

        normalized_pos = (self.current_position - self.min_value) / (
            self.max_value - self.min_value
        )
        self.current_effort = (1.0 - normalized_pos) * 10.0 + np.random.normal(0, 0.5)

    def create_joint_state_msg(self) -> JointState:
        """Create a JointState message with current gripper state."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        prefix = f"{self.namespace}_" if self.namespace else ""
        joint_name = f"{prefix}{self.joint_name}"

        msg.name = [joint_name]
        msg.position = [self.current_position]
        msg.velocity = [self.current_velocity]
        msg.effort = [self.current_effort]

        return msg

    def publish_joint_state(self):
        """Publish current gripper joint state."""
        self.update_gripper_state()

        joint_msg = self.create_joint_state_msg()
        self.joint_publisher.publish(joint_msg)

    def get_normalized_position(self) -> float:
        """Get gripper position normalized to [0, 1] range."""
        return (self.current_position - self.min_value) / (self.max_value - self.min_value)

    def is_open(self, threshold: float = 0.1) -> bool:
        """Check if gripper is open (above threshold)."""
        return self.get_normalized_position() > threshold

    def is_closed(self, threshold: float = 0.1) -> bool:
        """Check if gripper is closed (below threshold)."""
        return self.get_normalized_position() < threshold


def main(args=None):  # noqa: ANN001
    """Main entry point."""
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="Mock Gripper")
    parser.add_argument(
        "--command-topic",
        default="gripper_position_controller/commands",
        help="Command topic name",
    )
    parser.add_argument("--joint-topic", default="joint_states", help="Joint state topic name")
    parser.add_argument("--reboot-service", default="reboot_gripper", help="Reboot service name")
    parser.add_argument(
        "--torque-service",
        default="dynamixel_hardware_interface/set_dxl_torque",
        help="Enable torque service name",
    )
    parser.add_argument("--rate", type=float, default=30.0, help="Publishing rate (Hz)")
    parser.add_argument(
        "--min-value", type=float, default=0.0, help="Minimum gripper value (closed)"
    )
    parser.add_argument(
        "--max-value", type=float, default=0.08, help="Maximum gripper value (open)"
    )
    parser.add_argument("--joint-name", default="gripper_joint", help="Joint name")
    parser.add_argument("--namespace", default="", help="Gripper namespace")
    parser.add_argument("--index", type=int, default=0, help="Joint index in command array")

    parsed_args = parser.parse_args()

    node = MockGripperNode(
        command_topic=parsed_args.command_topic,
        joint_state_topic=parsed_args.joint_topic,
        reboot_service=parsed_args.reboot_service,
        enable_torque_service=parsed_args.torque_service,
        publish_rate=parsed_args.rate,
        min_value=parsed_args.min_value,
        max_value=parsed_args.max_value,
        joint_name=parsed_args.joint_name,
        namespace=parsed_args.namespace,
        index=parsed_args.index,
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
