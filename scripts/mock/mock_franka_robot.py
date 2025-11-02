#!/usr/bin/env python3
"""Mock Franka Robot that publishes pose, joint states, and twist to ROS2 topics."""

import argparse

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState


class MockFrankaRobotNode(Node):
    """Mock Franka robot node that publishes pose, joint states, and twist."""

    def __init__(
        self,
        current_pose_topic: str = "/current_pose",
        current_joint_topic: str = "/joint_states",
        current_twist_topic: str = "/current_twist",
        target_pose_topic: str = "/target_pose",
        target_joint_topic: str = "/target_joint",
        publish_rate: float = 50.0,
        base_frame: str = "base",
        target_frame: str = "fr3_hand_tcp",
        namespace: str = "",
    ):
        """Initialize the mock Franka robot node.

        Args:
            current_pose_topic: Topic name for publishing current pose
            current_joint_topic: Topic name for publishing joint states
            current_twist_topic: Topic name for publishing twist
            target_pose_topic: Topic name for subscribing to target pose
            target_joint_topic: Topic name for subscribing to target joint
            publish_rate: Publishing frequency in Hz
            base_frame: Base frame name
            target_frame: Target frame name
            namespace: Robot namespace
        """
        super().__init__("mock_franka_robot")

        self.base_frame = base_frame
        self.target_frame = target_frame
        self.namespace = namespace

        self.joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]

        self.home_config = np.array([0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4])

        # Current robot state (start at home)
        self.current_joints = self.home_config.copy()
        self.joint_velocities = np.zeros(7)
        self.current_pose = self._forward_kinematics(self.current_joints)
        self.current_twist = np.zeros(6)  # [vx, vy, vz, wx, wy, wz]

        # Target state (independent from current state)
        self.target_joints = self.home_config.copy()
        self.target_pose = self.current_pose.copy()

        # Control parameters
        self.joint_max_velocity = 0.5  # rad/s
        self.pose_max_velocity = 0.1  # m/s for position, rad/s for orientation

        # Create publishers
        self.pose_publisher = self.create_publisher(
            PoseStamped, current_pose_topic, qos_profile_sensor_data
        )
        self.joint_publisher = self.create_publisher(
            JointState, current_joint_topic, qos_profile_sensor_data
        )
        self.twist_publisher = self.create_publisher(
            TwistStamped, current_twist_topic, qos_profile_sensor_data
        )

        # Create subscribers for targets
        self.target_pose_subscriber = self.create_subscription(
            PoseStamped, target_pose_topic, self.target_pose_callback, qos_profile_system_default
        )
        self.target_joint_subscriber = self.create_subscription(
            JointState, target_joint_topic, self.target_joint_callback, qos_profile_system_default
        )

        # Create timer for publishing
        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.publish_data)

        self.get_logger().info(
            f"Mock Franka Robot in target following mode, publishing at {publish_rate} Hz with namespace '{namespace}'"
        )

    def _forward_kinematics(self, joint_angles: np.ndarray) -> dict:
        """Simplified forward kinematics for visualization purposes.

        This is a simplified approximation for demonstration.
        In a real scenario, you'd use proper DH parameters.
        """
        # Simplified kinematic chain - approximate Franka dimensions
        # This is just for visualization and doesn't match exact Franka kinematics
        link_lengths = [0.333, 0.0, 0.316, 0.0825, 0.384, 0.088, 0.107]

        # Start at base
        x, y, z = 0.0, 0.0, 0.0
        roll, pitch, yaw = 0.0, 0.0, 0.0

        # Simple kinematic approximation
        for i, (angle, length) in enumerate(zip(joint_angles, link_lengths)):
            if i % 2 == 0:  # Alternating joint axes
                yaw += angle
            else:
                pitch += angle

            # Add link contribution
            x += length * np.cos(yaw) * np.cos(pitch)
            y += length * np.sin(yaw) * np.cos(pitch)
            z += length * np.sin(pitch)

        return {"position": [x, y, z], "orientation": [roll, pitch, yaw]}

    def target_pose_callback(self, msg: PoseStamped):
        """Handle incoming target pose messages."""
        self.target_pose = {
            "position": [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            "orientation": self._quat_to_euler(
                [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ]
            ),
        }
        self.get_logger().debug(f"Received target pose: {self.target_pose}")

    def target_joint_callback(self, msg: JointState):
        """Handle incoming target joint messages."""
        if len(msg.position) >= 7:
            # Extract joint positions for this robot's joints
            prefix = f"{self.namespace}_" if self.namespace else ""
            for i, joint_name in enumerate(self.joint_names):
                full_name = f"{prefix}{joint_name}"
                if full_name in msg.name:
                    idx = msg.name.index(full_name)
                    self.target_joints[i] = msg.position[idx]
            self.get_logger().debug(f"Received target joints: {self.target_joints}")

    def _quat_to_euler(self, quat):  # noqa: ANN001, ANN202
        """Convert quaternion [x, y, z, w] to Euler angles [roll, pitch, yaw]."""
        r = Rotation.from_quat(quat)
        return r.as_euler("xyz")

    def _follow_targets(self, dt: float):
        """Make the robot smoothly follow target values."""
        joint_errors = self.target_joints - self.current_joints
        for i in range(7):
            max_delta = self.joint_max_velocity * dt
            delta = np.clip(joint_errors[i], -max_delta, max_delta)
            self.current_joints[i] += delta
            self.joint_velocities[i] = delta / dt

        pos_errors = np.array(self.target_pose["position"]) - np.array(
            self.current_pose["position"]
        )
        max_pos_delta = self.pose_max_velocity * dt
        for i in range(3):
            delta = np.clip(pos_errors[i], -max_pos_delta, max_pos_delta)
            self.current_pose["position"][i] += delta

        ori_errors = np.array(self.target_pose["orientation"]) - np.array(
            self.current_pose["orientation"]
        )
        max_ori_delta = self.pose_max_velocity * dt
        for i in range(3):
            delta = np.clip(ori_errors[i], -max_ori_delta, max_ori_delta)
            self.current_pose["orientation"][i] += delta

        self.current_twist[:3] = pos_errors / dt if np.linalg.norm(pos_errors) > 1e-6 else [0, 0, 0]
        self.current_twist[3:] = ori_errors / dt if np.linalg.norm(ori_errors) > 1e-6 else [0, 0, 0]

    def _create_pose_msg(self) -> PoseStamped:
        """Create a PoseStamped message from current pose."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame

        msg.pose.position.x = self.current_pose["position"][0]
        msg.pose.position.y = self.current_pose["position"][1]
        msg.pose.position.z = self.current_pose["position"][2]

        r = Rotation.from_euler("xyz", self.current_pose["orientation"])
        quat = r.as_quat()  # [x, y, z, w]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        return msg

    def _create_joint_msg(self) -> JointState:
        """Create a JointState message from current joint configuration."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        prefix = f"{self.namespace}_" if self.namespace else ""
        msg.name = [f"{prefix}{name}" for name in self.joint_names]
        msg.position = self.current_joints.tolist()
        msg.velocity = self.joint_velocities.tolist()
        msg.effort = [0.0] * 7  # Mock zero effort

        return msg

    def _create_twist_msg(self) -> TwistStamped:
        """Create a TwistStamped message from current twist."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame

        msg.twist.linear.x = self.current_twist[0]
        msg.twist.linear.y = self.current_twist[1]
        msg.twist.linear.z = self.current_twist[2]
        msg.twist.angular.x = self.current_twist[3]
        msg.twist.angular.y = self.current_twist[4]
        msg.twist.angular.z = self.current_twist[5]

        return msg

    def publish_data(self):
        """Publish robot state data."""
        dt = 1.0 / 50.0  # Time step for control loop

        # Follow target values
        self._follow_targets(dt)

        pose_msg = self._create_pose_msg()
        joint_msg = self._create_joint_msg()
        twist_msg = self._create_twist_msg()

        self.pose_publisher.publish(pose_msg)
        self.joint_publisher.publish(joint_msg)
        self.twist_publisher.publish(twist_msg)


def main(args=None):  # noqa: ANN001
    """Main entry point."""
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="Mock Franka Robot")
    parser.add_argument("--pose-topic", default="/current_pose", help="Current pose topic name")
    parser.add_argument("--joint-topic", default="/joint_states", help="Joint states topic name")
    parser.add_argument("--twist-topic", default="/current_twist", help="Current twist topic name")
    parser.add_argument(
        "--target-pose-topic", default="/target_pose", help="Target pose topic name"
    )
    parser.add_argument(
        "--target-joint-topic", default="/target_joint", help="Target joint topic name"
    )
    parser.add_argument("--rate", type=float, default=50.0, help="Publishing rate (Hz)")
    parser.add_argument("--base-frame", default="base", help="Base frame name")
    parser.add_argument("--target-frame", default="fr3_hand_tcp", help="Target frame name")
    parser.add_argument("--namespace", default="", help="Robot namespace")

    parsed_args = parser.parse_args()

    node = MockFrankaRobotNode(
        current_pose_topic=parsed_args.pose_topic,
        current_joint_topic=parsed_args.joint_topic,
        current_twist_topic=parsed_args.twist_topic,
        target_pose_topic=parsed_args.target_pose_topic,
        target_joint_topic=parsed_args.target_joint_topic,
        publish_rate=parsed_args.rate,
        base_frame=parsed_args.base_frame,
        target_frame=parsed_args.target_frame,
        namespace=parsed_args.namespace,
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
