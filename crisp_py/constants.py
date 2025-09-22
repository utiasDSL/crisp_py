"""Constants for CRISP component types and configurations."""

# Base constant tuples for reuse
COMPONENT_TYPES = ("robot", "camera", "gripper", "sensor")
ROBOT_TYPES = ("franka", "kinova", "iiwa", "so101", "generic", "custom")
CAMERA_TYPES = ("default", "custom")
GRIPPER_TYPES = ("default", "custom")
SENSOR_TYPES = ("empty", "float32", "force_torque", "custom")
CONFIGURATION_SOURCES = ("programmatic", "yaml_file", "default")

# Special type subsets for validation
CUSTOM_ROBOT_TYPES = ("generic", "custom")
CUSTOM_SENSOR_TYPES = ("custom",)
