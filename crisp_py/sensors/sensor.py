"""Contains objects to create sensor readers, basically objects that subscribe to a data stream topic."""

import threading

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

from crisp_py.sensors.sensor_config import SensorConfig


class Sensor:
    """Interface for sensor wrapper."""

    THREADS_REQUIRED = 1

    def __init__(
        self,
        node: Node | None = None,
        namespace: str = "",
        sensor_config: SensorConfig | None = None,
        spin_node: bool = True,
    ):
        """Initialize the gripper client.

        Args:
            node (Node, optional): ROS2 node to use. If None, creates a new node.
            namespace (str, optional): ROS2 namespace for the gripper.
            sensor_config (SensorConfig, optional): Configuration for the sensor.
            spin_node (bool, optional): Whether to spin the node in a separate thread.
        """
        if not rclpy.ok() and node is None:
            rclpy.init()

        self.config = sensor_config if sensor_config else SensorConfig()

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

        self.node.create_subscription(
            Float32MultiArray,
            self.config.data_topic,
            self._callback_sensor_data,
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )

        if spin_node:
            threading.Thread(target=self._spin_node, daemon=True).start()

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

    def _callback_sensor_data(self, msg: Float32MultiArray):
        """Callback for sensor data."""
        self._value = np.array(msg.data[:], dtype=np.float32)
        if self._baseline is None:
            self._baseline = np.zeros_like(self._value)
