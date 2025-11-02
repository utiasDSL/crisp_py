"""Contains objects to create sensor readers, basically objects that subscribe to a data stream topic."""

import threading
from abc import ABC, abstractmethod
from typing import Any, Callable

import numpy as np
import rclpy
import rclpy.subscription
import yaml
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node

from crisp_py.config.path import find_config, list_configs_in_folder
from crisp_py.sensors.sensor_config import SensorConfig, make_sensor_config
from crisp_py.utils import CallbackMonitor

sensor_registry: dict[str, type["Sensor"]] = {}


def register_sensor(sensor_type: str) -> Callable:
    """Decorator to register a sensor class in the sensor registry."""

    def decorator(sensor_cls: type["Sensor"]) -> type["Sensor"]:
        sensor_registry[sensor_type] = sensor_cls
        return sensor_cls

    return decorator


class Sensor(ABC):
    """Abstract base class for sensor wrappers."""

    THREADS_REQUIRED = 1

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
        self._baseline: np.ndarray | None = None
        self._callback_monitor = CallbackMonitor(
            self.node, stale_threshold=self.config.max_data_delay
        )

        self.sensor_subscriber = self._create_subscription()

        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

    @classmethod
    def from_yaml(
        cls,
        config_name: str,
        node: Node | None = None,
        namespace: str = "",
        spin_node: bool = True,
        **overrides,  # noqa: ANN003
    ) -> "Sensor":
        """Create a Sensor instance from a YAML configuration file.

        Args:
            config_name: Name of the config file (with or without .yaml extension)
            node: ROS2 node to use. If None, creates a new node.
            namespace: ROS2 namespace for the sensor.
            spin_node: Whether to spin the node in a separate thread.
            **overrides: Additional parameters to override YAML values

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

        sensor_config = make_sensor_config(**config_data)

        return _make_sensor_from_config(
            sensor_config=sensor_config,
            node=node,
            namespace=namespace,
            spin_node=spin_node,
        )

    @staticmethod
    def list_configs() -> list[str]:
        """List all available sensor configurations."""
        configs = list_configs_in_folder("sensors")
        return [config.stem for config in configs if config.suffix == ".yaml"]

    @abstractmethod
    def _create_subscription(self) -> rclpy.subscription.Subscription:
        """Create the ROS2 subscription for this sensor type."""
        pass

    @property
    def value(self) -> np.ndarray:
        """Get the latest calibrated sensor value."""
        if self._value is None or self._baseline is None:
            raise ValueError("Sensor value is not available yet.")
        return self._value - self._baseline

    @abstractmethod
    def ros_msg_to_sensor_value(self, msg: Any) -> np.ndarray:
        """Convert a ROS message to a numpy array.

        Args:
            msg: ROS message to convert

        Returns:
            np.ndarray: Converted numpy array
        """
        raise NotImplementedError("This method should be implemented in subclasses.")

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

    def calibrate_to_zero(self, num_samples: int = 10, sample_rate: float = 10.0):
        """Calibrate the sensor to zero.

        This function computes the average of a number of samples to compute the baseline.
        The value is then normalized by this average with the formula:

            calibrated_value = value - average(samples)

        Args:
            num_samples (int): Number of samples to take for calibration.
            sample_rate (float): Rate at which to take samples in Hz.
        """
        if self._value is None:
            raise ValueError("Sensor value is not available yet.")
        samples = np.zeros((num_samples, len(self._value)), dtype=np.float32)
        rate = self.node.create_rate(sample_rate)  # 10 Hz

        for sample_num in range(num_samples):
            samples[sample_num] = self.value
            rate.sleep()

        self._baseline = np.mean(samples, axis=0)

    def is_ready(self) -> bool:
        """Check if the sensor has a value."""
        return self._value is not None

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


def _make_sensor_from_config(
    sensor_config: SensorConfig,
    **kwargs,  # noqa: ANN003
) -> Sensor:
    """Internal factory function to create a sensor based on the configuration."""
    sensor_cls = sensor_registry.get(sensor_config.sensor_type)
    if sensor_cls is None:
        raise ValueError(f"Unknown sensor type: {sensor_config.sensor_type}")
    return sensor_cls(sensor_config=sensor_config, **kwargs)


def make_sensor(
    config_name: str | None = None,
    sensor_config: SensorConfig | None = None,
    node: Node | None = None,
    namespace: str = "",
    spin_node: bool = True,
    **overrides,  # noqa: ANN003
) -> Sensor:
    """Factory function to create a Sensor from a configuration file.

    Args:
        config_name: Name of the sensor config file
        sensor_config: Direct sensor config (if provided, config_name is ignored)
        node: ROS2 node to use. If None, creates a new node.
        namespace: ROS2 namespace for the sensor.
        spin_node: Whether to spin the node in a separate thread.
        **overrides: Additional parameters to override config values

    Returns:
        Sensor: Configured sensor instance

    Raises:
        FileNotFoundError: If the config file is not found
    """
    if sensor_config is not None:
        # Direct sensor config provided, use the internal factory function
        return _make_sensor_from_config(
            sensor_config=sensor_config,
            node=node,
            namespace=namespace,
            spin_node=spin_node,
        )
    elif config_name is not None:
        return Sensor.from_yaml(
            config_name=config_name,
            node=node,
            namespace=namespace,
            spin_node=spin_node,
            **overrides,
        )
    else:
        raise ValueError("Either config_name or sensor_config must be provided")


def list_sensor_configs() -> list[str]:
    """List all available sensor configurations."""
    return Sensor.list_configs()
