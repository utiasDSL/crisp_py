"""Provides a client to control the franka robot. It is the easiest way to control the robot using ROS2."""

import threading
from dataclasses import dataclass, field

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from crisp_py.control.controller_switcher import ControllerSwitcherClient
from crisp_py.control.joint_trajectory_controller_client import (
    JointTrajectoryControllerClient,
)
from crisp_py.gripper.gripper_client import GripperClient


@dataclass
class RobotConfig:
    base_frame: str
    target_frame: str
    home_config: list


@dataclass
class FrankaConfig(RobotConfig):
    base_frame: str = "base"
    # target_frame: str = "fr3_hand_tcp"
    target_frame: str = "fr3_link8"
    home_config: list = field(
        default_factory=lambda: [
            0,
            -np.pi / 4,
            0,
            -3 * np.pi / 4,
            0,
            np.pi / 2,
            np.pi / 4,
        ]
    )


class Robot:
    THREADS_REQUIRED = 4

    def __init__(
        self,
        node: Node = None,
        namespace: str = "",
        spin_node: bool = True,
        robot_config: RobotConfig = FrankaConfig(),
        prefix_frames: bool = True,
    ):
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node("robot_client", namespace=namespace)
        else:
            self.node = node
        self.robot_config = robot_config

        self._prefix = f"{namespace}_" if namespace != "" else ""

        self.controller_switcher_client = ControllerSwitcherClient(self.node)
        self.joint_trajectory_controller_client = JointTrajectoryControllerClient(
            self.node
        )
        self._target_pose_publisher = self.node.create_publisher(
            PoseStamped, "target_pose", qos_profile_system_default
        )

        # TF buffer to get the latest end effector pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self._from_frame = f"{self._prefix}{robot_config.target_frame}"
        self._to_frame = f"{self._prefix}{robot_config.base_frame}"
        self._end_effector_pose = None
        self._target_pose = None

        self.home_config = robot_config.home_config
        self.time_to_home = self.node.declare_parameter("time_to_home", 5.0).value
        self.publish_frequency = self.node.declare_parameter(
            "publish_frequency", 50.0
        ).value
        self.default_controller = self.node.declare_parameter(
            "default_controller", "cartesian_impedance_controller"
        ).value

        self.node.create_timer(
            1.0 / 10.0,
            self._callback_get_end_effector_pose,
            callback_group=ReentrantCallbackGroup(),
        )
        self.node.create_timer(
            1.0 / self.publish_frequency,
            self._callback_publish_target_pose,
            ReentrantCallbackGroup(),
        )
        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

    def _spin_node(self):
        if not rclpy.ok():
            rclpy.init()
        executor = rclpy.executors.MultiThreadedExecutor(
            num_threads=self.THREADS_REQUIRED
        )
        executor.add_node(self.node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    @property
    def end_effector_pose(self):
        return self._end_effector_pose

    def is_ready(self):
        """Returns True if the end-effector pose is available."""
        return self._end_effector_pose is not None

    def wait_until_ready(self, timeout: float = 10.0, check_frequency: float = 10.0):
        """Wait until the end-effector pose is available."""
        rate = self.node.create_rate(check_frequency)
        while not self.is_ready():
            rate.sleep()
            timeout -= 1.0 / check_frequency
            if timeout <= 0:
                raise TimeoutError("Timeout waiting for end-effector pose.")

    def set_target(self, position: iter = None, pose: pin.SE3 = None):
        """Sets directly the target pose of the end-effector to be published."""
        assert position is not None or pose is not None, (
            "Either position or pose must be provided."
        )

        desired_pose = (
            pose.copy() if pose is not None else self._end_effector_pose.copy()
        )
        if position is not None:
            assert len(position) == 3, "Position must be a 3D vector."
            desired_pose.translation = np.array(position)

        # TODO: Add some kind of check to avoid big jumps
        self._target_pose = pose.copy()

    def _callback_publish_target_pose(self):
        if self._target_pose is None:
            return
        self._target_pose_publisher.publish(self._pose_to_pose_msg(self._target_pose))

    def _callback_get_end_effector_pose(self):
        """Get the current pose of the end-effector."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self._to_frame, self._from_frame, rclpy.time.Time()
            )
            # self.node.get_logger().info(f"Got transform {transform}", throttle_duration_sec=1.0)
        except TransformException as ex:
            self.node.get_logger().warn(
                f"Could not transform {self._to_frame} to {self._from_frame}: {ex}",
                throttle_duration_sec=1.0,
            )
            return

        self._end_effector_pose = self._transform_to_pose(transform)
        if self._target_pose is None:
            self._target_pose = self._end_effector_pose
        return self._end_effector_pose

    def move_to(self, position: iter = None, pose: pin.SE3 = None, speed: float = 0.05):
        """Move the end-effector to a given pose.

        Args:
            position: Position to move to. If None, the pose is used.
            pose: The pose to move to. If None, the position is used.
            speed: The speed of the movement. [m/s]
        """
        assert position is not None or pose is not None, (
            "Either position or pose must be provided."
        )

        desired_pose = (
            pose.copy() if pose is not None else self._end_effector_pose.copy()
        )
        if position is not None:
            assert len(position) == 3, "Position must be a 3D vector."
            desired_pose.translation = np.array(position)

        start_pose = self._target_pose.copy()
        distance = np.linalg.norm(desired_pose.translation - start_pose.translation)
        time_to_move = distance / speed

        N = int(time_to_move * self.publish_frequency)

        rate = self.node.create_rate(self.publish_frequency)
        for t in np.linspace(0.0, 1.0, N):
            next_pose = pin.SE3.Interpolate(start_pose, desired_pose, t)
            self._target_pose = next_pose
            rate.sleep()

        self._target_pose = desired_pose

    def home(self, home_config=None, switch_to_default_controller: bool = True):
        """Home the robot."""
        self.controller_switcher_client.switch_controller("joint_trajectory_controller")
        self.joint_trajectory_controller_client.send_joint_config(
            self.home_config if home_config is None else home_config,
            self.time_to_home,
            blocking=True,
        )
        self._target_pose = None

        if switch_to_default_controller:
            self.controller_switcher_client.switch_controller(self.default_controller)

        # NOTE: Wait for the target pose to be set
        rate = self.node.create_rate(10)
        while self._target_pose is None:
            rate.sleep()
        rate.destroy()

    def _transform_to_pose(self, transform: TransformStamped) -> pin.SE3:
        """Convert a transform to a pose."""
        return pin.SE3(
            pin.Quaternion(
                x=transform.transform.rotation.x,
                y=transform.transform.rotation.y,
                z=transform.transform.rotation.z,
                w=transform.transform.rotation.w,
            ),
            np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ]),
        )

    def _pose_to_pose_msg(self, pose: pin.SE3) -> PoseStamped:
        """Convert a pose to a pose message."""
        pose_msg = PoseStamped()
        q = pin.Quaternion(pose.rotation)
        pose_msg.header.frame_id = self._to_frame
        pose_msg.pose.position.x = pose.translation[0]
        pose_msg.pose.position.y = pose.translation[1]
        pose_msg.pose.position.z = pose.translation[2]
        pose_msg.pose.orientation.x = q.x
        pose_msg.pose.orientation.y = q.y
        pose_msg.pose.orientation.z = q.z
        pose_msg.pose.orientation.w = q.w
        return pose_msg

    def shutdown(self):
        """Shutdown the node."""
        self.node.destroy_node()
        rclpy.shutdown()
