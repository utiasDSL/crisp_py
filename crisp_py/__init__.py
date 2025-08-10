"""Initialize crisp_py."""

from pathlib import Path

try:
    import rclpy  # noqa: F401
except ImportError:
    print("ROS2 should be installed and sourced!")


__version__ = "1.1.2"
