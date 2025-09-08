"""Utility functions for geometry operations."""

from dataclasses import dataclass

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, TwistStamped
from scipy.spatial.transform import Rotation


@dataclass
class Pose:
    """Compact representation of an SE3 object."""

    position: np.ndarray
    orientation: Rotation

    def copy(self) -> "Pose":
        """Create a copy of this pose."""
        return Pose(self.position.copy(), Rotation.from_quat(self.orientation.as_quat()))

    def __str__(self) -> str:
        """Return a string representation of a Pose."""
        return f"Pos: {np.array2string(self.position, suppress_small=True, precision=2, floatmode='fixed')},\n Orientation: {np.array2string(self.orientation.as_matrix(), suppress_small=True, precision=2, floatmode='fixed')}"

    def __sub__(self, other: "Pose") -> "Pose":
        """Subtract another pose from this pose, i.e. compute the relative pose."""
        return Pose(
            self.position - other.position,
            self.orientation * other.orientation.inv(),
        )

    def __add__(self, other: "Pose") -> "Pose":
        """Add another pose to this pose, i.e. add a relative pose."""
        return Pose(
            self.position + other.position,
            other.orientation * self.orientation,
        )

    @classmethod
    def from_ros_msg(cls, msg: PoseStamped) -> "Pose":
        """Create Pose from ROS PoseStamped message."""
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        orientation = Rotation.from_quat(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )
        return cls(position, orientation)

    def to_ros_msg(self, frame_id: str, stamp: TimeMsg) -> PoseStamped:
        """Convert Pose to ROS PoseStamped message."""
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.position
        q = self.orientation.as_quat()
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = q
        return msg


@dataclass
class Twist:
    """Compact representation of a twist (linear and angular velocity)."""

    linear: np.ndarray
    angular: np.ndarray

    def copy(self) -> "Twist":
        """Create a copy of this twist."""
        return Twist(self.linear.copy(), self.angular.copy())

    def __str__(self) -> str:
        """Return a string representation of a Twist."""
        return f"Linear: {np.array2string(self.linear, suppress_small=True, precision=2, floatmode='fixed')},\n Angular: {np.array2string(self.angular, suppress_small=True, precision=2, floatmode='fixed')}"

    def magnitude(self) -> float:
        """Return the magnitude of the linear velocity."""
        return np.linalg.norm(self.linear).astype(float)

    def angular_magnitude(self) -> float:
        """Return the magnitude of the angular velocity."""
        return np.linalg.norm(self.angular).astype(float)

    @classmethod
    def from_ros_msg(cls, msg: TwistStamped) -> "Twist":
        """Create Twist from ROS TwistStamped message."""
        linear = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        angular = np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z])
        return cls(linear, angular)

    def to_ros_msg(self, frame_id: str, stamp: TimeMsg) -> TwistStamped:
        """Convert Twist to ROS TwistStamped message."""
        msg = TwistStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = self.linear
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = self.angular
        return msg
