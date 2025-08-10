"""Initialize crisp_py."""

import importlib.metadata

__version__ = importlib.metadata.version("crisp_python")

try:
    import rclpy  # noqa: F401
except ImportError:
    print("ROS2 should be installed and sourced!")

try:
    import control_msgs  # noqa: F401

except ImportError:
    print(
        "Could not import control_msgs. Make sure that you installed ros-<ROS_DISTRO>-ros2-control(lers)."
    )
    print(
        "If you are using robostack with pixi, add the following lines to your pixi.toml dependencies:"
    )
    print('ros-<ROS_DISTRO>-ros2-control = "*"\nros-<ROS_DISTRO>-ros2-controllers = "*"')
