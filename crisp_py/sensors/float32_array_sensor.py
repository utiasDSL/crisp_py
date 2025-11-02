"""Sensor that subscribes to Float32MultiArray messages."""

import numpy as np
import rclpy
import rclpy.subscription
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from typing_extensions import override

from crisp_py.sensors.sensor import Sensor, register_sensor


@register_sensor("float32")
class Float32ArraySensor(Sensor):
    """Sensor that subscribes to Float32MultiArray messages."""

    @override
    def _create_subscription(self) -> rclpy.subscription.Subscription:
        """Create the ROS2 subscription for Float32MultiArray messages."""
        return self.node.create_subscription(
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

    @override
    def ros_msg_to_sensor_value(self, msg: Float32MultiArray) -> np.ndarray:
        """Convert a Float32MultiArray message to a numpy array.

        Args:
            msg: Float32MultiArray ROS message to convert

        Returns:
            np.ndarray: Converted numpy array
        """
        return np.array(msg.data[:], dtype=np.float32)
