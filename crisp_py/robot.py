"""Provides a client to control the franka robot. It is the easiest way to control the robot using ROS2."""

import threading

import numpy as np
import pinocchio as pin
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState

from crisp_py.control.controller_switcher import ControllerSwitcherClient
from crisp_py.control.joint_trajectory_controller_client import JointTrajectoryControllerClient
from crisp_py.control.parameters_client import ParametersClient
from crisp_py.robot_config import FrankaConfig, RobotConfig


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
        node: Node = None,
        namespace: str = "",
        spin_node: bool = True,
        robot_config: RobotConfig = None,
        prefix_frames: bool = True,
    ):
        """Initialize the robot interface.

        Args:
            node (Node, optional): ROS2 node to use. If None, creates a new node.
            namespace (str, optional): ROS2 namespace for the robot.
            spin_node (bool, optional): Whether to spin the node in a separate thread.
            robot_config (RobotConfig, optional): Robot configuration parameters.
            prefix_frames (bool, optional): Whether to prefix frame names with namespace.
        """
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node("robot_client", namespace=namespace)
        else:
            self.node = node
        self.config = robot_config if robot_config else FrankaConfig()

        self._prefix = f"{namespace}_" if namespace else ""

        self.controller_switcher_client = ControllerSwitcherClient(self.node)
        self.joint_trajectory_controller_client = JointTrajectoryControllerClient(self.node)
        self.cartesian_controller_parameters_client = ParametersClient(
            self.node, target_node=self.config.cartesian_impedance_controller_name
        )

        self._current_pose = None
        self._target_pose = None
        self._current_joint = None
        self._target_joint = None
        self._target_wrench = None

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
            self._callback_current_pose,
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )
        self.node.create_subscription(
            JointState,
            self.config.current_joint_topic,
            self._callback_current_joint,
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )

        self.node.create_timer(
            1.0 / self.config.publish_frequency,
            self._callback_publish_target_pose,
            ReentrantCallbackGroup(),
        )
        self.node.create_timer(
            1.0 / self.config.publish_frequency,
            self._callback_publish_target_joint,
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
    def nq(self):
        """Returns the number of joints of the robot."""
        return len(self.config.joint_names)

    @property
    def end_effector_pose(self):
        return self._current_pose

    @property
    def target_pose(self):
        return self._target_pose

    @property
    def joint_values(self):
        return self._current_joint.copy() if self._current_joint is not None else None

    def is_ready(self):
        """Returns True if the end-effector pose is available."""
        return self._current_pose is not None

    def reset_targets(self):
        """Reset the target pose, joint, and wrench to be None."""
        self._target_pose = None
        self._target_joint = None
        self._target_wrench = None

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
        target_pose = self._parse_pose_or_position(position, pose)
        self._target_pose = target_pose.copy()

    def set_target_joint(self, q: np.array):
        assert len(q) == self.nq, "Joint state must be of size nq."
        self._target_joint = q

    def _callback_publish_target_pose(self):
        if self._target_pose is None:
            return
        self._target_pose_publisher.publish(self._pose_to_pose_msg(self._target_pose))

    def _callback_publish_target_joint(self):
        if self._target_joint is None:
            return
        self._target_joint_publisher.publish(self._joint_to_joint_msg(self._target_joint))

    def _callback_publish_target_wrench(self):
        """Publish the target wrench if it exists."""
        if self._target_wrench is None:
            return
        self._target_wrench_publisher.publish(self._wrench_to_wrench_msg(self._target_wrench))

    def set_target_wrench(self, force: iter = None, torque: iter = None):
        """Sets the target wrench (force/torque) to be published.

        Args:
            force (iter, optional): Force vector [fx, fy, fz] in N. If None, zeros are used.
            torque (iter, optional): Torque vector [tx, ty, tz] in Nm. If None, zeros are used.
        """
        if force is None:
            force = [0.0, 0.0, 0.0]
        if torque is None:
            torque = [0.0, 0.0, 0.0]

        assert len(force) == 3, "Force must be a 3D vector"
        assert len(torque) == 3, "Torque must be a 3D vector"

        self._target_wrench = {"force": np.array(force), "torque": np.array(torque)}

    def _wrench_to_wrench_msg(self, wrench: dict) -> WrenchStamped:
        """Convert a wrench dictionary to a WrenchStamped message."""
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
        """Save the current pose."""
        self._current_pose = self._pose_msg_to_pose(msg)
        if self._target_pose is None:
            self._target_pose = self._current_pose.copy()

    def _callback_current_joint(self, msg: JointState):
        """Save the current joint state of the message by filtering the joint."""
        if self._current_joint is None:
            self._current_joint = np.zeros(self.nq)

        # self.node.get_logger().info(f"Current joint state: {msg.name} {msg.position}", throttle_duration_sec=1.0)
        for joint_name, joint_position in zip(msg.name, msg.position):
            if joint_name.removeprefix(self._prefix) not in self.config.joint_names:
                continue
            self._current_joint[
                self.config.joint_names.index(joint_name.removeprefix(self._prefix))
            ] = joint_position

    def move_to(self, position: iter = None, pose: pin.SE3 = None, speed: float = 0.05):
        """Move the end-effector to a given pose by interpolating linearly between the poses.

        Args:
            position: Position to move to. If None, the pose is used.
            pose: The pose to move to. If None, the position is used.
            speed: The speed of the movement. [m/s]
        """
        desired_pose = self._parse_pose_or_position(position, pose)
        start_pose = self._target_pose.copy()
        distance = np.linalg.norm(desired_pose.translation - start_pose.translation)
        time_to_move = distance / speed

        N = int(time_to_move * self.config.publish_frequency)

        rate = self.node.create_rate(self.config.publish_frequency)
        for t in np.linspace(0.0, 1.0, N):
            next_pose = pin.SE3.Interpolate(start_pose, desired_pose, t)
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

        # if switch_to_default_controller:
        #     self.controller_switcher_client.switch_controller(self.config.default_controller)

    def _pose_msg_to_pose(self, pose: PoseStamped) -> pin.SE3:
        """Convert a transform to a pose."""
        return pin.SE3(
            pin.Quaternion(
                x=pose.pose.orientation.x,
                y=pose.pose.orientation.y,
                z=pose.pose.orientation.z,
                w=pose.pose.orientation.w,
            ),
            np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]),
        )

    def _pose_to_pose_msg(self, pose: pin.SE3) -> PoseStamped:
        """Convert a pose to a pose message."""
        pose_msg = PoseStamped()
        q = pin.Quaternion(pose.rotation)
        pose_msg.header.frame_id = self.config.base_frame
        pose_msg.pose.position.x = pose.translation[0]
        pose_msg.pose.position.y = pose.translation[1]
        pose_msg.pose.position.z = pose.translation[2]
        pose_msg.pose.orientation.x = q.x
        pose_msg.pose.orientation.y = q.y
        pose_msg.pose.orientation.z = q.z
        pose_msg.pose.orientation.w = q.w
        return pose_msg

    def _joint_to_joint_msg(
        self, q: np.array, dq: np.array = None, tau: np.array = None
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

    def _parse_pose_or_position(self, position: iter = None, pose: pin.SE3 = None) -> pin.SE3:
        """Parse a pose from a desired position or pose.

        This function is a utility to create a pose object from either a position vector or a pose object.
        """
        assert position is not None or pose is not None, "Either position or pose must be provided."

        desired_pose = pose.copy() if pose is not None else self._target_pose.copy()
        if position is not None:
            assert len(position) == 3, "Position must be a 3D vector."
            desired_pose.translation = np.array(position)

        return desired_pose

    def shutdown(self):
        """Shutdown the node."""
        self.node.destroy_node()
        rclpy.shutdown()
