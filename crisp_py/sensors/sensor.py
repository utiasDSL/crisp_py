"""Contains objects to create sensor readers, basically objects that subscribe to a data stream topic."""

import threading
from abc import ABC
import time
from typing import Any, Callable

import numpy as np
import rclpy
import rclpy.subscription
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import MsgType, Node
from rclpy.qos import qos_profile_sensor_data

from crisp_py.config.path import find_config, list_configs_in_folder
from crisp_py.sensors.sensor_config import SensorConfig
from crisp_py.utils import CallbackMonitor
from crisp_py.utils.sliding_buffer import SlidingBuffer

"""Type alias for sensor specification."""
SensorSpec = tuple[MsgType, Callable[[MsgType], np.ndarray]]

"""Registry for sensor types and their corresponding specification functions."""
sensor_registry: dict[str, Callable] = {}


def register_sensor(sensor_type: str) -> Callable:
    """Decorator to register a sensor specification function.

    Args:
        sensor_type: Type of the sensor

    Returns:
        Callable: Decorator function
    """

    def decorator(func: Callable) -> Callable:
        sensor_registry[sensor_type] = func
        return func

    return decorator


def get_sensor_spec(sensor_type: str) -> SensorSpec:
    """Get the sensor specification for a given sensor type.

    Args:
        sensor_type: Type of the sensor

    Returns:
        SensorSpec: Tuple containing the ROS message type and conversion function

    Raises:
        ValueError: If the sensor type is unknown
    """
    sensor_spec_func = sensor_registry.get(sensor_type)
    if sensor_spec_func is None:
        raise ValueError(f"Unknown sensor type: {sensor_type}")
    return sensor_spec_func()


class Sensor(ABC):
    """Abstract base class for sensor wrappers."""

    THREADS_REQUIRED = 2

    def __init__(
        self,
        sensor_config: SensorConfig,
        node: Node | None = None,
        namespace: str = "",
        spin_node: bool = True,
    ):
        """Initialize the sensor.

        Args:
            sensor_config (SensorConfig, optional): Configuration for the sensor.
            node (Node, optional): ROS2 node to use. If None, creates a new node.
            namespace (str, optional): ROS2 namespace for the sensor.
            spin_node (bool, optional): Whether to spin the node in a separate thread.
        """
        if not rclpy.ok() and node is None:
            rclpy.init()

        self.config = sensor_config

        self.node = (
            rclpy.create_node(
                node_name=f"{self.config.name}_listener",
                namespace=namespace,
                parameter_overrides=[],
            )
            if not node
            else node
        )

        self._value: np.ndarray | None = None

        self._buffer = SlidingBuffer(
            size=self.config.buffer_size if self.config.buffer_size else 1,
            fill_value=np.zeros(self.config.shape, dtype=np.float32),
            buffer_type=np.ndarray,
        )

        self._callback_monitor = CallbackMonitor(
            self.node, stale_threshold=self.config.max_data_delay
        )

        self._sensor_msg_type, self.ros_msg_to_sensor_value = get_sensor_spec(
            self.config.sensor_type
        )

        self.sensor_subscriber = self.node.create_subscription(
            self._sensor_msg_type,
            self.config.data_topic,
            self._callback_monitor.monitor(
                name=f"{self.config.name}_monitor", func=self._sensor_callback
            ),
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )
        self.reset_client = (
            self.node.create_client(Trigger, self.config.reset_service)
            if self.config.reset_service
            else None
        )

        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

    def _sensor_callback(self, msg: Any):
        """Internal callback for sensor data subscription."""
        self._value = self.ros_msg_to_sensor_value(msg)
        self._buffer.add(self._value)

    @property
    def value(self) -> np.ndarray:
        """Get the latest sensor value."""
        if self._value is None:
            raise ValueError("Sensor value is not available yet.")
        return self._value

    @property
    def buffer(self) -> SlidingBuffer:
        """Get the sliding buffer of recent sensor values."""
        return self._buffer

    def _spin_node(self):
        if not rclpy.ok():
            rclpy.init()
        executor = (
            MultiThreadedExecutor(num_threads=self.THREADS_REQUIRED)
            if self.THREADS_REQUIRED > 1
            else SingleThreadedExecutor()
        )
        executor.add_node(self.node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    def is_ready(self) -> bool:
        """Check if the sensor has a value."""
        return self._value is not None and self._reset_service_is_available()

    def _reset_service_is_available(self) -> bool:
        """Check if the reset service is available, if configured."""
        if self.reset_client is None:
            return True
        return self.reset_client.service_is_ready()

    def reset(self, timeout: float = 5.0):
        """Reset the sensor by calling the reset service if configured.

        This method will call the reset service (if configured) and clear the buffer.
        If the service call times out, a warning is logged but the method does not fail.
        If no reset service is configured, the method silently clears the buffer.

        Args:
            timeout (float): Timeout in seconds for the service call. Default is 5.0 seconds.
        """
        if self.reset_client is not None:
            request = Trigger.Request()
            future = self.reset_client.call_async(request)
            start_time = time.time()
            while rclpy.ok():
                time.sleep(0.1)
                if future.done():
                    try:
                        response = future.result()
                        if response is None:
                            self.node.get_logger().warning(
                                "Reset service call returned None response"
                            )
                        elif not response.success:
                            self.node.get_logger().warning(
                                f"Reset service call returned success=False: {response.message}"
                            )
                    except Exception as e:
                        self.node.get_logger().warning(
                            f"Reset service call failed with exception: {e}"
                        )
                    break

                elapsed = time.time() - start_time
                if elapsed >= timeout:
                    self.node.get_logger().warning(f"Reset service call timed out after {timeout}s")
                    break

        self._buffer = SlidingBuffer(
            size=self.config.buffer_size if self.config.buffer_size else 1,
            fill_value=np.zeros(self.config.shape, dtype=np.float32),
            buffer_type=np.ndarray,
        )

    def wait_until_ready(self, timeout: float = 10.0, check_frequency: float = 10.0):
        """Wait until the gripper is available."""
        rate = self.node.create_rate(check_frequency)
        while not self.is_ready():
            rate.sleep()
            timeout -= 1.0 / check_frequency
            if timeout <= 0:
                raise TimeoutError(
                    f"Timeout waiting for sensor to be ready. Is the topic {self.config.data_topic} for the namespace {self.node.get_namespace()} being published to?"
                )


def make_sensor(
    config_name: str,
    node: Node | None = None,
    namespace: str = "",
    spin_node: bool = True,
    **overrides,  # noqa: ANN003
) -> Sensor:
    """Factory function to create a Sensor from a configuration file.

    Args:
        config_name: Name of the sensor config file or path to a custom config file.
        node: ROS2 node to use. If None, creates a new node.
        namespace: ROS2 namespace for the sensor.
        spin_node: Whether to spin the node in a separate thread.
        **overrides: Additional parameters to override config values

    Returns:
        Sensor: Configured sensor instance

    Raises:
        FileNotFoundError: If the config file is not found
    """
    if not config_name.endswith(".yaml"):
        config_name += ".yaml"

    config_path = find_config(f"sensors/{config_name}")
    if config_path is None:
        config_path = find_config(config_name)

    if config_path is None:
        raise FileNotFoundError(
            f"Sensor config file '{config_name}' not found in any CRISP config paths"
        )

    with open(config_path, "r") as f:
        data = yaml.safe_load(f) or {}

    data.update(overrides)

    namespace = data.pop("namespace", namespace)
    config_data = data.pop("sensor_config", data)

    sensor_config = SensorConfig(**config_data)

    return Sensor(
        sensor_config=sensor_config,
        node=node,
        namespace=namespace,
        spin_node=spin_node,
    )


def list_sensor_configs() -> list[str]:
    """List all available sensor configurations."""
    configs = list_configs_in_folder("sensors")
    return [config.stem for config in configs if config.suffix == ".yaml"]
