"""Force torque sensor that subscribes to WrenchStamped messages."""

import numpy as np
import rclpy
import rclpy.subscription
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import WrenchStamped
from typing_extensions import override

from crisp_py.sensors.sensor import Sensor, register_sensor


@register_sensor("force_torque")
class ForceTorqueSensor(Sensor):
    """Torque sensor that subscribes to WrenchStamped messages."""

    @override
    def _create_subscription(self) -> rclpy.subscription.Subscription:
        """Create the ROS2 subscription for WrenchStamped messages."""
        return self.node.create_subscription(
            WrenchStamped,
            self.config.data_topic,
            self._callback_monitor.monitor(name="force_torque_monitor", func=self._callback_wrench),
            qos_profile_sensor_data,
            callback_group=ReentrantCallbackGroup(),
        )

    def _callback_wrench(self, msg: WrenchStamped):
        """Callback for wrench data."""
        self._value = self.ros_msg_to_sensor_value(msg)
        if self._baseline is None:
            self._baseline = np.zeros_like(self._value)

    @override
    def ros_msg_to_sensor_value(self, msg: WrenchStamped) -> np.ndarray:
        """Convert a WrenchStamped message to a numpy array.

        Args:
            msg: WrenchStamped ROS message to convert

        Returns:
            np.ndarray: Converted numpy array
        """
        return np.array(
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
