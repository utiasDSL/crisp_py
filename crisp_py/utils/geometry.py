"""Utility functions for geometry operations."""

from dataclasses import dataclass
from enum import Enum

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, TwistStamped
from scipy.spatial.transform import Rotation
from tf2_ros import TransformStamped


class OrientationRepresentation(Enum):
    """Enum for different orientation representations."""

    QUATERNION = "quaternion"
    EULER = "euler"
    ANGLE_AXIS = "angle_axis"


@dataclass
class Pose:
    """Compact representation of an SE3 object."""

    position: np.ndarray
    orientation: Rotation

    def __post_init__(self):
        """Validate the inputs."""
        if self.position.shape != (3,):
            raise ValueError("Position must be a 3D vector.")
        if not isinstance(self.orientation, Rotation):
            raise ValueError("Orientation must be a scipy Rotation object.")

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
        quat = np.array(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )
        quat_x_sign = np.sign(quat[0]) if quat[0] != 0 else 1.0
        quat *= quat_x_sign  # Ensure a consistent quaternion sign
        orientation = Rotation.from_quat(quat)
        return cls(position, orientation)

    @classmethod
    def from_transform_msg(cls, msg: TransformStamped) -> "Pose":
        """Create Pose from ROS TransformStamped message."""
        position = np.array(
            [msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z]
        )
        quat = np.array(
            [
                msg.transform.rotation.x,
                msg.transform.rotation.y,
                msg.transform.rotation.z,
                msg.transform.rotation.w,
            ]
        )
        quat_x_sign = np.sign(quat[0]) if quat[0] != 0 else 1.0
        quat *= quat_x_sign  # Ensure a consistent quaternion sign
        orientation = Rotation.from_quat(quat)
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

    def to_pos_euler_array(self) -> np.ndarray:
        """Convert Pose to a 6D array (position + Euler angles)."""
        euler = self.orientation.as_euler("xyz", degrees=False)
        return np.concatenate([self.position, euler], axis=0)

    def to_pos_quat_array(self) -> np.ndarray:
        """Convert Pose to a 7D array (position + quaternion)."""
        quat = self.orientation.as_quat()
        return np.concatenate([self.position, quat], axis=0)

    def to_pos_angle_axis_array(self) -> np.ndarray:
        """Convert Pose to a 6D array (position + angle-axis)."""
        angle_axis = self.orientation.as_rotvec()
        return np.concatenate([self.position, angle_axis], axis=0)

    def to_array(
        self, representation: OrientationRepresentation = OrientationRepresentation.EULER
    ) -> np.ndarray:
        """Convert Pose to an array with the specified orientation representation.

        Args:
            representation: The orientation representation to use. One of
                OrientationRepresentation. Defaults to OrientationRepresentation.EULER.

        Returns:
            A numpy array representing the pose.
        """
        if representation == OrientationRepresentation.EULER:
            return self.to_pos_euler_array()
        elif representation == OrientationRepresentation.QUATERNION:
            return self.to_pos_quat_array()
        elif representation == OrientationRepresentation.ANGLE_AXIS:
            return self.to_pos_angle_axis_array()
        else:
            raise ValueError(f"Unknown orientation representation: {representation}")


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
