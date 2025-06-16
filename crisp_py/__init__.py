"""Initialize crisp_py."""

try:
    import rclpy  # noqa: F401
except ImportError:
    print("ROS2 should be installed and sourced!")

__version__ = "1.0.11"
