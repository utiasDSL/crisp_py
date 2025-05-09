"""Generic class for a gripper based on a simple ros2 topic."""

import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from crisp_py.control.controller_switcher import ControllerSwitcherClient


class GripperConfig:
    command_topic: str = "position_controller/commands"
    joint_state_topic: str = "gripper/joint_state"
    min_width: float = 0.0
    max_width: float = 0.08  # [m]


class Gripper:
    """Interface for gripper wrapper."""

    def __init__(
        self,
        node: Node = None,
        namespace: str = "",
        config: GripperConfig = None,
        spin_node: bool = True,
        gripper_config=None,
    ):
        """Initialize the gripper client.

        Args:
            node (Node, optional): ROS2 node to use. If None, creates a new node.
            namespace (str, optional): ROS2 namespace for the gripper.
            spin_node (bool, optional): Whether to spin the node in a separate thread.
        """
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            self.node = rclpy.create_node("gripper_client", namespace=namespace)
        else:
            self.node = node

        self._prefix = f"{namespace}_" if namespace else ""

        self.controller_switcher_client = ControllerSwitcherClient(self.node)
        # self.gripper_parameter_client = ParametersClient(
        #     self.node, target_node=todo
        # )

        self._command_publisher = self.node.create_publisher(
            Float64MultiArray,
            self.config.command_topic,
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
    def min_width(self) -> float:
        """Returns the minimum width of the gripper."""
        return self.config.min_width

    @property
    def max_width(self) -> float:
        """Returns the minimum width of the gripper."""
        return self.config.max_width

    @property
    def width(self) -> float | None:
        """Returns the current width of the gripper or None if not initialized."""
        return

    @property
    def is_ready(self) -> bool:
        """Returns True if the gripper is fully ready to operate."""
        return self.width is not None

    def wait_until_ready(self, timeout: float = 10.0, check_frequency: float = 10.0):
        """Wait until the gripper is available."""
        rate = self.node.create_rate(check_frequency)
        while not self.is_ready():
            rate.sleep()
            timeout -= 1.0 / check_frequency
            if timeout <= 0:
                raise TimeoutError("Timeout waiting for gripper to be ready.")

    def set_target(
        self,
        target: float,
    ):
        """Grasp with the gripper by setting a target. This can be a position, velocity or effort depending on the active controller.
        Args:
            width (float): The width of the gripper.
        """
        msg = Float64MultiArray()
        msg.data = [target]
        self._command_publisher.publish(msg)
