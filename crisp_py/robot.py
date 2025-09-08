"""Provides a client to control the franka robot. It is the easiest way to control the robot using ROS2."""

import threading
from typing import List

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from numpy.typing import NDArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import JointState

from crisp_py.control.controller_switcher import ControllerSwitcherClient
from crisp_py.control.joint_trajectory_controller_client import JointTrajectoryControllerClient
from crisp_py.control.parameters_client import ParametersClient
from crisp_py.robot_config import FrankaConfig, RobotConfig
from crisp_py.utils.callback_monitor import CallbackMonitor
from crisp_py.utils.geometry import Pose, Twist


class Robot:
    """A high-level interface for controlling robots using ROS2.

    This class provides an easy-to-use interface for controlling robots through ROS2,
    supporting both joint space and Cartesian space control. It handles controller
    switching, trajectory generation, and state monitoring.

    Attributes:
        THREADS_REQUIRED (int): Number of threads required for the ROS2 executor
        node (Node): ROS2 node instance
        config (RobotConfig): Robot configuration parameters
        controller_switcher_client: Client for switching between controllers
        joint_trajectory_controller_client: Client for joint trajectory control
        cartesian_controller_parameters_client: Client for Cartesian controller parameters
    """

    THREADS_REQUIRED = 4

    def __init__(
        self,
        node: Node | None = None,
        namespace: str = "",
        spin_node: bool = True,
        robot_config: RobotConfig | None = None,
        name: str = "robot_client",
    ) -> None:
        """Initialize the robot interface.

        Args:
            node (Node, optional): ROS2 node to use. If None, creates a new node.
            namespace (str, optional): ROS2 namespace for the robot.
            spin_node (bool, optional): Whether to spin the node in a separate thread.
            robot_config (RobotConfig, optional): Robot configuration parameters.
            name (str, optional): Name of the robot client node.
        """
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node(name, namespace=namespace)
        else:
            self.node = node
        self.config = robot_config if robot_config else FrankaConfig()

        self._prefix = f"{namespace}_" if namespace else ""

        self.controller_switcher_client = ControllerSwitcherClient(self.node)
        self.joint_trajectory_controller_client = JointTrajectoryControllerClient(self.node)
        self.cartesian_controller_parameters_client = ParametersClient(
            self.node, target_node=self.config.cartesian_impedance_controller_name
        )
        self.joint_controller_parameters_client = ParametersClient(
            self.node, target_node=self.config.joint_trajectory_controller_name
        )

        self._current_pose = None
        self._target_pose = None
        self._current_joint = None
        self._target_joint = None
        self._target_wrench = None
        self._current_twist = None

        self._callback_monitor = CallbackMonitor(
            node=self.node,
            stale_threshold=max(self.config.max_pose_delay, self.config.max_joint_delay),
        )

        self._target_pose_publisher = self.node.create_publisher(
            PoseStamped, self.config.target_pose_topic, qos_profile_system_default
        )
        self._target_wrench_publisher = self.node.create_publisher(
            WrenchStamped, "target_wrench", qos_profile_system_default
        )
        self._target_joint_publisher = self.node.create_publisher(
            JointState, self.config.target_joint_topic, qos_profile_system_default
        )
        self.node.create_subscription(
            PoseStamped,
            self.config.current_pose_topic,
            self._callback_monitor.monitor(
                f"{namespace.capitalize()} Current Pose", self._callback_current_pose
            ),
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )
        self.node.create_subscription(
            JointState,
            self.config.current_joint_topic,
            self._callback_monitor.monitor(
                f"{namespace.capitalize()} Current Joint", self._callback_current_joint
            ),
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )

        self.node.create_subscription(
            TwistStamped,
            self.config.current_twist_topic,
            self._callback_monitor.monitor(
                f"{namespace.capitalize()} Current Twist", self._callback_current_twist
            ),
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )

        self.node.create_timer(
            1.0 / self.config.publish_frequency,
            self._callback_monitor.monitor(
                f"{namespace.capitalize()} Target Pose", self._callback_publish_target_pose
            ),
            ReentrantCallbackGroup(),
        )
        self.node.create_timer(
            1.0 / self.config.publish_frequency,
            self._callback_monitor.monitor(
                f"{namespace.capitalize()} Target Joint", self._callback_publish_target_joint
            ),
            ReentrantCallbackGroup(),
        )
        self.node.create_timer(
            1.0 / self.config.publish_frequency,
            self._callback_publish_target_wrench,
            ReentrantCallbackGroup(),
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
    def nq(self) -> int:
        """Get the number of joints in the robot.

        Returns:
            int: The number of joints in the robot configuration.
        """
        return len(self.config.joint_names)

    @property
    def end_effector_pose(self) -> Pose:
        """Get the current pose of the end effector.

        Returns:
            Pose: The current pose of the end effector, or None if not available.
        """
        if self._current_pose is None:
            raise RuntimeError(
                "The robot has not received any poses yet. Run wait_until_ready() before running anything else."
            )
        return self._current_pose.copy()

    @property
    def target_pose(self) -> Pose:
        """Get the target pose of the end effector.

        Returns:
            Pose: The target pose of the end effector, or None if not set.
        """
        if self._target_pose is None:
            raise RuntimeError(
                "The robot has not received any poses yet. Run wait_until_ready() before running anything else."
            )
        return self._target_pose.copy()

    @property
    def joint_values(self) -> NDArray:
        """Get the current joint values of the robot.

        Returns:
            numpy.ndarray: Copy of current joint values, or None if not available.
        """
        if self._current_joint is None:
            raise RuntimeError(
                "The robot has not received any joints yet. Run wait_until_ready() before running anything else."
            )
        return self._current_joint.copy()

    @property
    def target_joint(self) -> NDArray:
        """Get the target joint values of the robot.

        Returns:
            numpy.ndarray: Copy of target joint values, or None if not available.
        """
        if self._target_joint is None:
            raise RuntimeError(
                "The robot has not received any joints yet. Run wait_until_ready() before running anything else."
            )
        return self._target_joint.copy()

    @property
    def end_effector_twist(self) -> Twist:
        """Get the current twist of the end effector.

        Returns:
            Twist: Current end-effector twist message.

        Raises:
            RuntimeError: If no twist messages have been received yet.
        """
        if self._current_twist is None:
            raise RuntimeError(
                f"The robot has not received any twists yet. Is the current_twist_topic {self.config.current_twist_topic} correct?"
            )
        return self._current_twist.copy()

    def is_ready(self) -> bool:
        """Check if the robot is ready for operation.

        Returns:
            bool: True if all necessary values for operation are available, False otherwise.
        """
        return (
            self._current_pose is not None
            and self._target_pose is not None
            and self._current_joint is not None
            and self._target_joint is not None
        )

    def reset_targets(self):
        """Reset all target values to None.

        This method clears the target pose, joint values, and wrench values,
        effectively stopping any ongoing movement or force application.
        """
        self._target_pose = None
        self._target_joint = None
        self._target_wrench = None

    def wait_until_ready(self, timeout: float = 10.0, check_frequency: float = 10.0):
        """Wait until the robot is ready for operation.

        Args:
            timeout (float, optional): Maximum time to wait in seconds. Defaults to 10.0.
            check_frequency (float, optional): How often to check readiness in Hz. Defaults to 10.0.

        Raises:
            TimeoutError: If the robot is not ready within the specified timeout.
        """
        rate = self.node.create_rate(check_frequency)
        while not self.is_ready():
            rate.sleep()
            timeout -= 1.0 / check_frequency
            if timeout <= 0:
                raise TimeoutError("Timeout waiting for end-effector pose.")

    def set_target(self, position: List | NDArray | None = None, pose: Pose | None = None):
        """Set the target pose for the end-effector.

        Args:
            position (List | NDArray, optional): Target position as [x, y, z]. If None, uses current orientation.
            pose (Pose, optional): Target pose as SE3 transform. If None, uses position.

        Note:
            Either position or pose must be provided. If both are provided, position overrides
            the translation component of pose.
        """
        target_pose = self._parse_pose_or_position(position, pose)
        self._target_pose = target_pose.copy()

    def set_target_joint(self, q: NDArray):
        """Set the target joint configuration.

        Args:
            q (np.array): Target joint values array of size nq.

        Raises:
            AssertionError: If q is not the same size as the number of joints.
        """
        assert len(q) == self.nq, "Joint state must be of size nq."
        self._target_joint = q

    def _callback_publish_target_pose(self):
        """Publish the current target pose if one exists.

        This callback is triggered periodically to publish the target pose
        to the ROS topic for the robot controller.
        """
        if self._target_pose is None or not rclpy.ok():
            return
        self._target_pose_publisher.publish(
            self._target_pose.to_ros_msg(
                self.config.base_frame, self.node.get_clock().now().to_msg()
            )
        )

    def _callback_publish_target_joint(self):
        """Publish the current target joint configuration if one exists.

        This callback is triggered periodically to publish the target joint values
        to the ROS topic for the robot controller.
        """
        if self._target_joint is None or not rclpy.ok():
            return
        self._target_joint_publisher.publish(self._joint_to_joint_msg(self._target_joint))

    def _callback_publish_target_wrench(self):
        """Publish the target wrench if one exists.

        This callback is triggered periodically to publish the target wrench (force/torque)
        to the ROS topic for the robot controller.
        """
        if self._target_wrench is None or not rclpy.ok():
            return
        self._target_wrench_publisher.publish(self._wrench_to_wrench_msg(self._target_wrench))

    def set_target_wrench(
        self, force: List | NDArray | None = None, torque: List | NDArray | None = None
    ):
        """Set the target wrench (force/torque) to be applied by the robot.

        Args:
            force (list, optional): Force vector [fx, fy, fz] in N. If None, zeros are used.
            torque (list, optional): Torque vector [tx, ty, tz] in Nm. If None, zeros are used.

        Raises:
            AssertionError: If force or torque vectors are not 3D vectors.
        """
        if force is None:
            force = [0.0, 0.0, 0.0]
        if torque is None:
            torque = [0.0, 0.0, 0.0]

        assert len(force) == 3, "Force must be a 3D vector"
        assert len(torque) == 3, "Torque must be a 3D vector"

        self._target_wrench = {"force": np.array(force), "torque": np.array(torque)}

    def _wrench_to_wrench_msg(self, wrench: dict) -> WrenchStamped:
        """Convert a wrench dictionary to a ROS WrenchStamped message.

        Args:
            wrench (dict): Dictionary containing 'force' and 'torque' numpy arrays.

        Returns:
            WrenchStamped: ROS message containing the wrench data with proper header.
        """
        wrench_msg = WrenchStamped()
        wrench_msg.header.frame_id = self.config.base_frame
        wrench_msg.header.stamp = self.node.get_clock().now().to_msg()
        wrench_msg.wrench.force.x = wrench["force"][0]
        wrench_msg.wrench.force.y = wrench["force"][1]
        wrench_msg.wrench.force.z = wrench["force"][2]
        wrench_msg.wrench.torque.x = wrench["torque"][0]
        wrench_msg.wrench.torque.y = wrench["torque"][1]
        wrench_msg.wrench.torque.z = wrench["torque"][2]
        return wrench_msg

    def _callback_current_pose(self, msg: PoseStamped):
        """Update the current pose from a ROS message.

        This callback is triggered when a new pose message is received. It updates
        the current pose and initializes the target pose if not already set.

        Args:
            msg (PoseStamped): ROS message containing the current pose.
        """
        self._current_pose = Pose.from_ros_msg(msg)
        if self._target_pose is None:
            self._target_pose = self._current_pose.copy()

    def _callback_current_twist(self, msg: TwistStamped):
        """Update the current twist from a ROS message.

        This callback is triggered when a new twist message is received. It updates
        the current twist.

        Args:
            msg (TwistStamped): ROS message containing the current twist.
        """
        self._current_twist = Twist.from_ros_msg(msg)

    def _callback_current_joint(self, msg: JointState):
        """Update the current joint state from a ROS message.

        This callback filters the joint state message to only include joints
        that are part of this robot's configuration.

        Args:
            msg (JointState): ROS message containing joint states.
        """
        if self._current_joint is None:
            self._current_joint = np.zeros(self.nq)

        # self.node.get_logger().info(f"Current joint state: {msg.name} {msg.position}", throttle_duration_sec=1.0)
        for joint_name, joint_position in zip(msg.name, msg.position):
            if joint_name.removeprefix(self._prefix) not in self.config.joint_names:
                continue
            self._current_joint[
                self.config.joint_names.index(joint_name.removeprefix(self._prefix))
            ] = joint_position

        if self._target_joint is None:
            self._target_joint = self._current_joint.copy()

    def move_to(
        self, position: List | NDArray | None = None, pose: Pose | None = None, speed: float = 0.05
    ):
        """Move the end-effector to a given pose by interpolating linearly between the poses.

        Args:
            position: Position to move to. If None, the pose is used.
            pose: The pose to move to. If None, the position is used.
            speed: The speed of the movement. [m/s]
        """
        if self._current_pose is None:
            raise RuntimeError(
                "The robot has not received any poses yet. Run wait_until_ready() before running anything else."
            )
        desired_pose = self._parse_pose_or_position(position, pose)
        start_pose = self._current_pose
        distance = np.linalg.norm(desired_pose.position - start_pose.position)
        time_to_move = distance / speed

        N = int(time_to_move * self.config.publish_frequency)

        rate = self.node.create_rate(self.config.publish_frequency)

        slerp = Slerp(
            [0, 1], Rotation.concatenate([start_pose.orientation, desired_pose.orientation])
        )

        for t in np.linspace(0.0, 1.0, N):
            pos = (1 - t) * start_pose.position + t * desired_pose.position
            ori = slerp([t])[0]
            next_pose = Pose(pos, ori)
            self._target_pose = next_pose
            rate.sleep()

        self._target_pose = desired_pose

    def home(self, home_config: list[float] | None = None, blocking: bool = True):
        """Home the robot."""
        self.controller_switcher_client.switch_controller("joint_trajectory_controller")
        self.joint_trajectory_controller_client.send_joint_config(
            self.config.joint_names,
            self.config.home_config if home_config is None else home_config,
            self.config.time_to_home,
            blocking=blocking,
        )

        # Set to none to avoid publishing the previous target pose after activating the next controller
        self._target_pose = None
        self._target_joint = None

        if blocking:
            self.wait_until_ready()

        # if switch_to_default_controller:
        #     self.controller_switcher_client.switch_controller(self.config.default_controller)

    def _joint_to_joint_msg(
        self, q: NDArray, dq: NDArray | None = None, tau: NDArray | None = None
    ) -> JointState:
        """Convert a pose to a pose message."""
        joint_msg = JointState()
        joint_msg.header.frame_id = self.config.base_frame
        joint_msg.header.stamp = self.node.get_clock().now().to_msg()
        joint_msg.name = [self._prefix + joint_name for joint_name in self.config.joint_names]
        joint_msg.position = q.tolist()
        joint_msg.velocity = dq.tolist() if dq is not None else [0.0] * len(q)
        joint_msg.effort = tau.tolist() if tau is not None else [0.0] * len(q)
        return joint_msg

    def _parse_pose_or_position(
        self, position: List | NDArray | None = None, pose: Pose | None = None
    ) -> Pose:
        """Parse a pose from a desired position or pose.

        This function is a utility to create a pose object from either a position vector or a pose object.
        """
        assert position is not None or pose is not None, "Either position or pose must be provided."

        desired_pose = pose.copy() if pose is not None else self.target_pose
        if position is not None:
            assert len(position) == 3, "Position must be a 3D vector."
            desired_pose.position = np.array(position)

        return desired_pose

    def is_homed(self) -> bool:
        """Check if the robot is homed.

        This method checks if the robot's current joint configuration matches the home configuration.

        Returns:
            bool: True if the robot is homed, False otherwise.
        """
        return np.allclose(self.joint_values, self.config.home_config, atol=1e-3)

    def shutdown(self):
        """Shutdown the node."""
        if rclpy.ok():
            rclpy.shutdown()
