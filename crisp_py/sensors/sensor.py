"""Contains objects to create sensor readers, basically objects that subscribe to a data stream topic."""

import threading
from abc import ABC, abstractmethod

import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

from crisp_py.sensors.sensor_config import SensorConfig
from crisp_py.utils import CallbackMonitor


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

        self._create_subscription()

        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

    @abstractmethod
    def _create_subscription(self):
        """Create the ROS2 subscription for this sensor type."""
        pass

    @property
    def value(self) -> np.ndarray:
        """Get the latest calibrated sensor value."""
        if self._value is None or self._baseline is None:
            raise ValueError("Sensor value is not available yet.")
        return self._value - self._baseline

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
                raise TimeoutError("Timeout waiting for sensor to be ready.")


class Float32ArraySensor(Sensor):
    """Sensor that subscribes to Float32MultiArray messages."""

    def _create_subscription(self):
        """Create the ROS2 subscription for Float32MultiArray messages."""
        self.node.create_subscription(
            Float32MultiArray,
            self.config.data_topic,
            self._callback_monitor.monitor(name="float32", func=self._callback_sensor_data),
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )

    def _callback_sensor_data(self, msg: Float32MultiArray):
        """Callback for sensor data."""
        self._value = np.array(msg.data[:], dtype=np.float32)
        if self._baseline is None:
            self._baseline = np.zeros_like(self._value)


class ForceTorqueSensor(Sensor):
    """Torque sensor that subscribes to WrenchStamped messages."""

    def _create_subscription(self):
        """Create the ROS2 subscription for WrenchStamped messages."""
        self.node.create_subscription(
            WrenchStamped,
            self.config.data_topic,
            self._callback_monitor.monitor(name="force_torque_monitor", func=self._callback_wrench),
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )

    def _callback_wrench(self, msg: WrenchStamped):
        """Callback for wrench data."""
        self._value = np.array(
            [
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
            ],
            dtype=np.float32,
        )
        if self._baseline is None:
            self._baseline = np.zeros_like(self._value)


def make_sensor(
    sensor_config: SensorConfig,
    **kwargs,  # noqa: ANN003
) -> Sensor:
    """Factory function to create a sensor based on the configuration."""
    if sensor_config.sensor_type == "float32":
        return Float32ArraySensor(
            sensor_config=sensor_config,
            **kwargs,
        )
    elif sensor_config.sensor_type == "force_torque":
        return ForceTorqueSensor(
            sensor_config=sensor_config,
            **kwargs,
        )
    raise ValueError(f"Unknown sensor type: {sensor_config.sensor_type}")
