"""Sensor specification for Float32MultiArray sensors."""

import numpy as np
from std_msgs.msg import Float32MultiArray

from crisp_py.sensors.sensor import SensorSpec, register_sensor


@register_sensor("float32_array")
def get_float32_array_sensor_spec() -> SensorSpec:
    """Get the sensor specification for Float32MultiArray sensors.

    Returns:
        SensorSpec: Tuple containing the ROS message type and conversion function
    """
    return (
        Float32MultiArray,
        lambda msg: np.array(msg.data[:], dtype=np.float32),
    )
