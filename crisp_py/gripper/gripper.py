"""Generic class for a gripper based on a simple ros2 topic."""

import threading
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from crisp_py.utils.callback_monitor import CallbackMonitor


@dataclass
class GripperConfig:
    """Gripper default config.

    Can be extented to be used with other grippers.
    """

    min_value: float
    max_value: float
    command_topic: str = "gripper_position_controller/commands"
    joint_state_topic: str = "joint_states"
    reboot_service: str = "reboot_gripper"
    enable_torque_service: str = "dynamixel_hardware_interface/set_dxl_torque"
    index: int = 0
    publish_frequency: float = 30.0
    max_joint_delay: float = 1.0
    max_delta: float = 0.1

    @classmethod
    def from_yaml(cls, path: str | Path) -> "GripperConfig":
        """Create a GripperConfig from a YAML configuration file.

        Args:
            path (str | Path): Path to the YAML configuration file from the project root or directly full path from the filesystem.
        """
        if isinstance(path, str):
            project_root_path = Path(__file__).parent.parent.parent
            full_path = project_root_path / path
        elif isinstance(path, Path):
            full_path = path
        else:
            raise TypeError("Path must be a string or a Path object.")

        with open(full_path, "r") as file:
            config = yaml.safe_load(file)
            config = {
                "min_value": config.get("min_value", 0.0),
                "max_value": config.get("max_value", 1.0),
                "command_topic": config.get(
                    "command_topic", "gripper_position_controller/commands"
                ),
                "joint_state_topic": config.get("joint_state_topic", "joint_states"),
                "reboot_service": config.get("reboot_service", "reboot_gripper"),
                "enable_torque_service": config.get(
                    "enable_torque_service", "dynamixel_hardware_interface/set_dxl_torque"
                ),
                "index": config.get("index", 0),
                "publish_frequency": config.get("publish_frequency", 30.0),
                "max_delta": config.get("max_delta", 0.1),
                "max_joint_delay": config.get("max_joint_delay", 1.0),
            }
        return cls(**config)


class Gripper:
    """Interface for gripper wrapper."""

    THREADS_REQUIRED = 2

    def __init__(
        self,
        node: Node | None = None,
        namespace: str = "",
        gripper_config: GripperConfig | None = None,
        spin_node: bool = True,
    ):
        """Initialize the gripper client.

        Args:
            node (Node, optional): ROS2 node to use. If None, creates a new node.
            namespace (str, optional): ROS2 namespace for the gripper.
            gripper_config (GripperConfig, optional): configuration for the gripper class.
            spin_node (bool, optional): Whether to spin the node in a separate thread.
        """
        if not rclpy.ok() and node is None:
            rclpy.init()

        self.node = (
            rclpy.create_node(
                node_name="gripper_client", namespace=namespace, parameter_overrides=[]
            )
            if not node
            else node
        )
        self.config = (
            gripper_config if gripper_config else GripperConfig(min_value=0.0, max_value=1.0)
        )

        self._prefix = f"{namespace}_" if namespace else ""
        self._value = None
        self._torque = None
        self._target = None
        self._index = self.config.index
        self._callback_monitor = CallbackMonitor(
            self.node, stale_threshold=self.config.max_joint_delay
        )

        self._command_publisher = self.node.create_publisher(
            Float64MultiArray,
            self.config.command_topic,
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )
        self.node.create_subscription(
            JointState,
            self.config.joint_state_topic,
            self._callback_monitor.monitor(
                f"{namespace.capitalize()} Gripper Joint State", self._callback_joint_state
            ),
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )

        self.node.create_timer(
            1.0 / self.config.publish_frequency,
            self._callback_monitor.monitor(
                f"{namespace.capitalize()} Gripper Target Publisher", self._callback_publish_target
            ),
            ReentrantCallbackGroup(),
        )

        self.reboot_client = self.node.create_client(Trigger, self.config.reboot_service)
        self.enable_torque_client = self.node.create_client(
            SetBool, self.config.enable_torque_service
        )

        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

    def _spin_node(self):
        if not rclpy.ok():
            rclpy.init()
        executor = MultiThreadedExecutor(num_threads=self.THREADS_REQUIRED)
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
    def is_valid(self) -> bool:
        """Returns true if the joint values received are valid."""
        if self._value is None:
            self.node.get_logger().error(
                f"{self._prefix}Gripper is not initialized. Call wait_until_ready() first."
            )
            return False
        if self._normalize(self._value) > 1.05 or self._normalize(self.value) < -0.05:
            self.node.get_logger().error(
                f"{self._prefix}Gripper value {self._value} is out of bounds [0.0, 1.0]. Please check the gripper configuration, and eventually calibrate the gripper."
            )
            self.node.get_logger().error(
                f"The raw value of the gripper is {self.raw_value}, which is not in the range [{self.min_value}, {self.max_value}]."
            )
            return False
        return True

    @property
    def value(self) -> float | None:
        """Returns the current value of the gripper or None if not initialized."""
        if self._value is None:
            raise RuntimeError(
                f"{self._prefix}Gripper is not initialized. Call wait_until_ready() first."
            )
        # Check if joint state callback is stale
        namespace_part = self._prefix.rstrip("_").capitalize() if self._prefix else ""
        callback_name = f"{namespace_part} Gripper Joint State".strip()
        try:
            joint_callback_data = self._callback_monitor.get_callback_data(callback_name)
            if joint_callback_data and joint_callback_data.is_stale:
                self.node.get_logger().warn(f"{self._prefix}Gripper joint state is stale")
        except ValueError:
            # Callback not found, which is expected if no data has been received yet
            pass
        return np.clip(self._normalize(self._value), 0.0, 1.0)

    @property
    def raw_value(self) -> float | None:
        """Returns the current raw value of the gripper or None if not initialized."""
        return self._value

    def is_ready(self) -> bool:
        """Returns True if the gripper is fully ready to operate."""
        return self._value is not None

    def wait_until_ready(self, timeout: float = 10.0, check_frequency: float = 10.0):
        """Wait until the gripper is available."""
        rate = self.node.create_rate(check_frequency)
        while not self.is_ready():
            rate.sleep()
            timeout -= 1.0 / check_frequency
            if timeout <= 0:
                raise TimeoutError("Timeout waiting for gripper to be ready.")

    def is_open(self, open_threshold: float = 0.1) -> bool:
        """Returns True if the gripper is open."""
        if self.value is None:
            raise RuntimeError("Gripper value is not initialized. Call wait_until_ready() first.")
        return self.value > open_threshold

    def close(self):
        """Close the gripper."""
        self.set_target(target=0.0)

    def open(self):
        """Open the gripper."""
        self.set_target(target=1.0)

    def _callback_publish_target(self):
        """Publish the target command."""
        if self._target is None:
            return
        msg = Float64MultiArray()
        msg.data = [
            self._unnormalize(
                self.value
                + np.clip(
                    self._normalize(self._target) - self.value,
                    -self.config.max_delta,
                    self.config.max_delta,
                )
            )
        ]
        self._command_publisher.publish(msg)

    def _callback_joint_state(self, msg: JointState):
        """Save the latest joint state values.

        Note: we assume that the gripper value is the first element of the joint message.

        Args:
            msg (JointState): the message containing the joint state.
        """
        self._value = msg.position[self._index]
        self._torque = msg.effort[self._index]

    def set_target(self, target: float, *, epsilon: float = 0.1):
        """Grasp with the gripper by setting a target. This can be a position, velocity or effort depending on the active controller.

        Args:
            target (float): The target value for the gripper between 0 and 1 from closed to open respectively.
            epsilon (float): allowed zone around the target limits that are allowed to be set.
        """
        assert 0.0 - epsilon <= target <= 1.0 + epsilon, (
            f"The target should be normalized between 0 and 1, but is currently {target}"
        )
        self._target = self._unnormalize(target)

    def _normalize(self, unormalized_value: float) -> float:
        """Normalize a raw value between 0.0 and 1.0."""
        return (unormalized_value - self.min_value) / (self.max_value - self.min_value)

    def _unnormalize(self, normalized_value: float) -> float:
        """Normalize a raw value between 0.0 and 1.0."""
        return (self.max_value - self.min_value) * normalized_value + self.min_value

    def shutdown(self):
        """Shutdown the node and allow the robot to be instantiated again."""
        if rclpy.ok():
            rclpy.shutdown()

    def reboot(self, block: bool = False):
        """Reboot the gripper if the reboot service is available.

        Args:
            block: if block is set to True, then we wait until a response arrives.
        """
        if not self.reboot_client.service_is_ready:
            raise RuntimeError(
                f"Trying to reboot the client but the service {self.config.reboot_service} is not available. Is the gripper running? Does your gripper support rebooting?"
            )

        if block:
            self.reboot_client.call(Trigger.Request())
        else:
            self.reboot_client.call_async(Trigger.Request())

    def enable_torque(self, block: bool = False):
        """Enable torque holding in the gripper.

        Args:
            block: if block is set to True, then we wait until a response arrives.
        """
        self._set_torque_holding(enable=True, block=block)

    def disable_torque(self, block: bool = False):
        """Disable torque holding in the gripper to allow free movement.

        Args:
            block: if block is set to True, then we wait until a response arrives.
        """
        self._set_torque_holding(enable=False, block=block)

    def _set_torque_holding(self, enable: bool, block: bool = False):
        """Reboot the gripper if the reboot service is available.

        Args:
            enable: whether or not we enable the torque holding in the motor.
            block: if block is set to True, then we wait until a response arrives.
        """
        if not self.enable_torque_client.service_is_ready:
            raise RuntimeError(
                f"Trying to enable torque the client but the service {self.config.enable_torque_service} is not available. Is the gripper running? Does your gripper support toggling reboot?"
            )

        req = SetBool.Request()
        req.data = enable
        if block:
            self.enable_torque_client.call(req)
        else:
            self.enable_torque_client.call_async(req)
