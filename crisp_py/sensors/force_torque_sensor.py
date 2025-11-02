"""Module for force-torque sensor specifications."""

import numpy as np
from geometry_msgs.msg import WrenchStamped

from crisp_py.sensors.sensor import SensorSpec, register_sensor


@register_sensor("force_torque")
def get_force_torque_sensor_spec() -> SensorSpec:
    """Get the sensor specification for force-torque sensors.

    Returns:
        SensorSpec: Tuple containing the ROS message type and conversion function.
    """
    return (
        WrenchStamped,
        lambda msg: np.array(
            [
                msg.wrench.force.x,
                msg.wrench.force.y,
                msg.wrench.force.z,
                msg.wrench.torque.x,
                msg.wrench.torque.y,
                msg.wrench.torque.z,
            ],
            dtype=np.float32,
        ),
    )
