"""Example demonstrating the CRISP factory pattern for creating components."""

import logging
from crisp_py import CrispFactory
from crisp_py.robot_config import FrankaConfig
from crisp_py.camera.camera_config import CameraConfig
from crisp_py.gripper.gripper import GripperConfig
from crisp_py.sensors.sensor_config import ForceTorqueSensorConfig

# Set up logging to see factory config resolution
logging.basicConfig(level=logging.INFO)

# Create factory instance
factory = CrispFactory()

# ========================
# 1. Using Default Configs
# ========================
print("=== Using Default Configs ===")

# Default Franka robot
robot = factory.create_robot(robot_type="franka")

# Default camera
camera = factory.create_camera()

# Default gripper
gripper = factory.create_gripper()

# Force-torque sensor (no default, would need config)
# sensor = factory.create_sensor(sensor_type="force_torque")

print("Created components with default configs")

# ===================================
# 2. Programmatic Config Registration
# ===================================
print("\n=== Programmatic Config Registration ===")

# Register custom Franka config
custom_franka = FrankaConfig(
    home_config=[0, -0.5, 0, -2.5, 0, 2.0, 0.8],
    publish_frequency=100.0,
    time_to_home=3.0
)

factory.register_config(
    component_type="robot",
    type_name="franka",
    config_name="lab_setup",
    config_instance=custom_franka
)

# Register custom camera config
custom_camera = CameraConfig(
    camera_name="overhead_cam",
    resolution=(640, 480),
    camera_color_image_topic="overhead_camera/color/image_raw",
    camera_color_info_topic="overhead_camera/color/camera_info"
)

factory.register_config(
    component_type="camera",
    type_name="default",
    config_name="overhead",
    config_instance=custom_camera
)

# Register custom gripper config
precision_gripper = GripperConfig(
    min_value=0.0,
    max_value=0.08,
    command_topic="precision_gripper/commands",
    max_delta=0.02
)

factory.register_config(
    component_type="gripper",
    type_name="default",
    config_name="precision",
    config_instance=precision_gripper
)

# Register custom sensor config
ft_sensor = ForceTorqueSensorConfig(
    name="wrist_ft_sensor",
    data_topic="wrist_wrench"
)

factory.register_config(
    component_type="sensor",
    type_name="force_torque",
    config_name="wrist_sensor",
    config_instance=ft_sensor
)

# Create components using registered configs
robot_custom = factory.create_robot(
    robot_type="franka",
    config_name="lab_setup",
    namespace="left_arm"
)

camera_custom = factory.create_camera(
    config_name="overhead",
    namespace="overhead"
)

gripper_custom = factory.create_gripper(
    config_name="precision",
    namespace="left"
)

sensor_custom = factory.create_sensor(
    sensor_type="force_torque",
    config_name="wrist_sensor"
)

print("Created components with programmatic configs")

# =====================================
# 3. Custom Robot Types (Generic/Custom)
# =====================================
print("\n=== Custom Robot Types ===")

# For custom robots, you must provide programmatic config or YAML
# This would fail: factory.create_robot(robot_type="custom")

# Register config for a custom robot
from crisp_py.robot_config import RobotConfig

custom_robot_config = RobotConfig(
    joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    home_config=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    base_frame="custom_base",
    target_frame="custom_end_effector",
    current_joint_topic="custom_robot/joint_states",
    current_pose_topic="custom_robot/pose"
)

factory.register_config(
    component_type="robot",
    type_name="my_custom_robot",
    config_name="lab_config",
    config_instance=custom_robot_config
)

# Now create the custom robot
custom_robot = factory.create_robot(
    robot_type="my_custom_robot",
    config_name="lab_config"
)

print("Created custom robot with programmatic config")

# =========================
# 4. YAML-based Configs
# =========================
print("\n=== YAML-based Configs (would work if YAML files exist) ===")

# These would work if corresponding YAML files exist in config paths:
# robot_yaml = factory.create_robot(robot_type="franka", config_name="production_setup")
# camera_yaml = factory.create_camera(config_name="high_resolution")
# gripper_yaml = factory.create_gripper(config_name="soft_grasp")

print("YAML configs would be loaded from files in CRISP_CONFIG_PATH")

# ===============================
# 5. Backward Compatibility
# ===============================
print("\n=== Backward Compatibility ===")

# Old way still works
from crisp_py.robot import Robot
from crisp_py.robot_config import FrankaConfig
from crisp_py.camera.camera import Camera

old_robot = Robot(robot_config=FrankaConfig())
old_camera = Camera()

print("Old-style instantiation still works")

print("\n=== Factory Pattern Demo Complete ===")
print("Check the logs above to see where each config was resolved from!")