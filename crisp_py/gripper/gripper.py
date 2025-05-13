"""Generic class for a gripper based on a simple ros2 topic."""

import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# from crisp_py.control.controller_switcher import ControllerSwitcherClient


class GripperConfig:
    command_topic: str = "gripper_position_controller/commands"
    joint_state_topic: str = "joint_states"

    def __init__(self, min_value, max_value):
        self.min_value = min_value
        self.max_value = max_value


class Gripper:
    """Interface for gripper wrapper."""

    THREADS_REQUIRED = 2

    def __init__(
        self,
        node: Node = None,
        namespace: str = "",
        gripper_config: GripperConfig = None,
        spin_node: bool = True,
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
        self.config = gripper_config if gripper_config else GripperConfig(min_value=0.0, max_value=1.0)

        self._prefix = f"{namespace}_" if namespace else ""
        self._value = None
        self._torque = None

        # self.controller_switcher_client = ControllerSwitcherClient(self.node)
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
            self.config.joint_state_topic,
            self._callback_joint_state,
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )

        # self.node.create_timer(
        #     1.0 / self.config.publish_frequency,
        #     self._callback_publish_target,
        #     ReentrantCallbackGroup(),
        # )

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
    def min_value(self) -> float:
        """Returns the minimum width of the gripper."""
        return self.config.min_value

    @property
    def max_value(self) -> float:
        """Returns the minimum width of the gripper."""
        return self.config.max_value

    @property
    def torque(self) -> float | None:
        """Returns the current torque of the gripper or None if not initialized."""
        return self._torque

    @property
    def value(self) -> float | None:
        """Returns the current value of the gripper or None if not initialized."""
        return self._normalize(self._value) if self._value is not None else None

    @property
    def raw_value(self) -> float | None:
        """Returns the current raw value of the gripper or None if not initialized."""
        return self._value

    def is_ready(self) -> bool:
        """Returns True if the gripper is fully ready to operate."""
        return self.value is not None

    def wait_until_ready(self, timeout: float = 10.0, check_frequency: float = 10.0):
        """Wait until the gripper is available."""
        rate = self.node.create_rate(check_frequency)
        while not self.is_ready():
            rate.sleep()
            timeout -= 1.0 / check_frequency
            if timeout <= 0:
                raise TimeoutError("Timeout waiting for gripper to be ready.")

    def _callback_joint_state(self, msg: JointState):
        """TODO"""
        self._value = msg.position[0]
        self._torque = msg.effort[0]

    def set_target(
        self,
        target: float,
        *,
        epsilon: float = 0.1,
    ):
        """Grasp with the gripper by setting a target. This can be a position, velocity or effort depending on the active controller.
        Args:
            width (float): The width of the gripper.
        """
        assert 0.0 - epsilon <= target <= 1.0 + epsilon, (
            f"The target should be normalized between 0 and 1, but is currently {target}"
        )
        msg = Float64MultiArray()
        msg.data = [self._unnormalize(target)]
        self._command_publisher.publish(msg)

    def _normalize(self, unormalized_value: float):
        """Normalize a raw value between 0.0 and 1.0."""
        return (unormalized_value - self.min_value) / (self.max_value - self.min_value)

    def _unnormalize(self, normalized_value: float):
        """Normalize a raw value between 0.0 and 1.0."""
        return (self.max_value - self.min_value) * normalized_value + self.min_value

    def shutdown(self):
        self.node.destroy_node()
