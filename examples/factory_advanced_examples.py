"""Advanced examples demonstrating the CRISP factory pattern capabilities."""

import logging
from pathlib import Path
from crisp_py import CrispFactory
from crisp_py.robot_config import FrankaConfig, RobotConfig
from crisp_py.camera.camera_config import CameraConfig
from crisp_py.gripper.gripper import GripperConfig
from crisp_py.sensors.sensor_config import ForceTorqueSensorConfig, SensorConfig

# Set up logging
logging.basicConfig(level=logging.INFO)

def demo_multi_robot_setup():
    """Demonstrate setting up multiple robots with different configurations."""
    print("\n=== Multi-Robot Setup Demo ===")
    
    factory = CrispFactory()
    
    # Register configs for left arm
    left_arm_config = FrankaConfig(
        home_config=[0.0, -0.5, 0.0, -2.5, 0.0, 2.0, 0.8],
        base_frame="left_arm_base",
        target_frame="left_arm_ee",
        current_joint_topic="left_arm/joint_states",
        current_pose_topic="left_arm/current_pose",
        target_pose_topic="left_arm/target_pose",
        target_joint_topic="left_arm/target_joint"
    )
    
    factory.register_config(
        component_type="robot",
        type_name="franka",
        config_name="left_arm",
        config_instance=left_arm_config
    )
    
    # Register configs for right arm  
    right_arm_config = FrankaConfig(
        home_config=[0.0, 0.5, 0.0, -2.5, 0.0, 2.0, -0.8],
        base_frame="right_arm_base",
        target_frame="right_arm_ee", 
        current_joint_topic="right_arm/joint_states",
        current_pose_topic="right_arm/current_pose",
        target_pose_topic="right_arm/target_pose",
        target_joint_topic="right_arm/target_joint"
    )
    
    factory.register_config(
        component_type="robot",
        type_name="franka",
        config_name="right_arm",
        config_instance=right_arm_config
    )
    
    # Test config resolution
    left_config = factory._resolve_config("robot", "franka", "left_arm")
    right_config = factory._resolve_config("robot", "franka", "right_arm")
    
    assert left_config.base_frame == "left_arm_base"
    assert right_config.base_frame == "right_arm_base"
    assert left_config.home_config[-1] == 0.8
    assert right_config.home_config[-1] == -0.8
    
    print("✓ Multi-robot setup successful")
    return factory

def demo_sensor_ecosystem():
    """Demonstrate setting up multiple sensor types."""
    print("\n=== Sensor Ecosystem Demo ===")
    
    factory = CrispFactory()
    
    # Force-torque sensor
    ft_config = ForceTorqueSensorConfig(
        name="wrist_ft",
        data_topic="robot/wrist_wrench"
    )
    
    factory.register_config(
        component_type="sensor",
        type_name="force_torque",
        config_name="wrist",
        config_instance=ft_config
    )
    
    # Custom sensor type
    custom_sensor_config = SensorConfig(
        shape=(5,),
        sensor_type="float32",
        name="tactile_sensor",
        data_topic="robot/tactile_data"
    )
    
    factory.register_config(
        component_type="sensor", 
        type_name="custom",
        config_name="fingertip_tactile",
        config_instance=custom_sensor_config
    )
    
    # Test sensor config resolution
    ft_resolved = factory._resolve_config("sensor", "force_torque", "wrist")
    custom_resolved = factory._resolve_config("sensor", "custom", "fingertip_tactile")
    
    assert ft_resolved.name == "wrist_ft"
    assert ft_resolved.data_topic == "robot/wrist_wrench"
    assert custom_resolved.name == "tactile_sensor"
    
    print("✓ Sensor ecosystem setup successful")
    return factory

def demo_camera_configurations():
    """Demonstrate different camera configurations."""
    print("\n=== Camera Configuration Demo ===")
    
    factory = CrispFactory()
    
    # High-resolution overhead camera
    overhead_config = CameraConfig(
        camera_name="overhead_view",
        camera_frame="overhead_camera_link",
        resolution=(1920, 1080),
        camera_color_image_topic="overhead/color/image_raw",
        camera_color_info_topic="overhead/color/camera_info"
    )
    
    factory.register_config(
        component_type="camera",
        type_name="default",
        config_name="overhead_hd",
        config_instance=overhead_config
    )
    
    # Eye-in-hand camera
    eih_config = CameraConfig(
        camera_name="eye_in_hand",
        camera_frame="camera_optical_frame",
        resolution=(640, 480),
        camera_color_image_topic="camera/color/image_raw",
        camera_color_info_topic="camera/color/camera_info",
        max_image_delay=0.1  # Tighter timing for robot control
    )
    
    factory.register_config(
        component_type="camera",
        type_name="default",
        config_name="eye_in_hand",
        config_instance=eih_config
    )
    
    # Test camera configs
    overhead_resolved = factory._resolve_config("camera", "default", "overhead_hd")
    eih_resolved = factory._resolve_config("camera", "default", "eye_in_hand")
    
    assert overhead_resolved.resolution == (1920, 1080)
    assert eih_resolved.max_image_delay == 0.1
    assert overhead_resolved.camera_name == "overhead_view"
    
    print("✓ Camera configuration demo successful")
    return factory

def demo_gripper_precision_settings():
    """Demonstrate different gripper precision configurations."""
    print("\n=== Gripper Precision Demo ===")
    
    factory = CrispFactory()
    
    # High-precision gripper for small objects
    precision_config = GripperConfig(
        min_value=0.0,
        max_value=0.04,  # 4cm max opening
        command_topic="precision_gripper/command",
        max_delta=0.005  # 5mm max change per command
    )
    
    factory.register_config(
        component_type="gripper",
        type_name="default",
        config_name="micro_precision",
        config_instance=precision_config
    )
    
    # Heavy-duty gripper for large objects
    heavy_duty_config = GripperConfig(
        min_value=0.0,
        max_value=0.15,  # 15cm max opening
        command_topic="heavy_gripper/command", 
        max_delta=0.05   # 5cm max change per command
    )
    
    factory.register_config(
        component_type="gripper",
        type_name="default",
        config_name="heavy_duty",
        config_instance=heavy_duty_config
    )
    
    # Test gripper configs
    precision_resolved = factory._resolve_config("gripper", "default", "micro_precision")
    heavy_resolved = factory._resolve_config("gripper", "default", "heavy_duty")
    
    assert precision_resolved.max_value == 0.04
    assert precision_resolved.max_delta == 0.005
    assert heavy_resolved.max_value == 0.15
    assert heavy_resolved.max_delta == 0.05
    
    print("✓ Gripper precision demo successful")
    return factory

def demo_custom_robot_integration():
    """Demonstrate integrating a completely custom robot."""
    print("\n=== Custom Robot Integration Demo ===")
    
    factory = CrispFactory()
    
    # 4-DOF SCARA robot configuration
    scara_config = RobotConfig(
        joint_names=["joint1", "joint2", "joint3", "joint4"],
        home_config=[0.0, 0.0, 0.0, 0.0],
        base_frame="scara_base",
        target_frame="scara_tool",
        current_joint_topic="scara/joint_states",
        current_pose_topic="scara/current_pose",
        target_pose_topic="scara/target_pose",
        target_joint_topic="scara/target_joint",
        publish_frequency=25.0,  # Lower frequency for SCARA
        time_to_home=8.0
    )
    
    factory.register_config(
        component_type="robot",
        type_name="scara_custom",
        config_name="lab_scara",
        config_instance=scara_config
    )
    
    # 3-DOF delta robot configuration
    delta_config = RobotConfig(
        joint_names=["arm1", "arm2", "arm3"],
        home_config=[0.0, 0.0, 0.0],
        base_frame="delta_base",
        target_frame="delta_effector",
        current_joint_topic="delta/joints", 
        current_pose_topic="delta/pose",
        target_pose_topic="delta/target_pose",
        target_joint_topic="delta/target_joints",
        publish_frequency=200.0,  # High frequency for delta
        time_to_home=2.0
    )
    
    factory.register_config(
        component_type="robot",
        type_name="delta_custom",
        config_name="pick_place_delta",
        config_instance=delta_config
    )
    
    # Test custom robot configs
    scara_resolved = factory._resolve_config("robot", "scara_custom", "lab_scara")
    delta_resolved = factory._resolve_config("robot", "delta_custom", "pick_place_delta")
    
    assert len(scara_resolved.joint_names) == 4
    assert len(delta_resolved.joint_names) == 3
    assert scara_resolved.target_frame == "scara_tool"
    assert delta_resolved.publish_frequency == 200.0
    
    print("✓ Custom robot integration demo successful")
    return factory

def demo_configuration_priorities():
    """Demonstrate the configuration priority system."""
    print("\n=== Configuration Priority Demo ===")
    
    factory = CrispFactory()
    
    # Register a programmatic config
    programmatic_config = FrankaConfig(
        home_config=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        time_to_home=10.0
    )
    
    factory.register_config(
        component_type="robot",
        type_name="franka",
        config_name="priority_test",
        config_instance=programmatic_config
    )
    
    # Test priority 1: Programmatic (highest)
    config = factory._resolve_config("robot", "franka", "priority_test")
    assert config.time_to_home == 10.0
    print("✓ Priority 1 (programmatic): config.time_to_home =", config.time_to_home)
    
    # Test priority 3: Default (lowest)
    config = factory._resolve_config("robot", "franka", None)
    assert config.time_to_home != 10.0  # Should be default value
    print("✓ Priority 3 (default): config.time_to_home =", config.time_to_home)
    
    # Test nonexistent config name falls back to default
    config = factory._resolve_config("robot", "franka", "nonexistent")
    assert config.time_to_home != 10.0  # Should be default value
    print("✓ Nonexistent config falls back to default")
    
    print("✓ Configuration priority system working correctly")
    return factory

def run_all_demos():
    """Run all factory pattern demos."""
    print("=== CRISP Factory Pattern - Advanced Examples ===")
    
    demo_multi_robot_setup()
    demo_sensor_ecosystem()
    demo_camera_configurations()
    demo_gripper_precision_settings()
    demo_custom_robot_integration()
    demo_configuration_priorities()
    
    print("\n=== All Advanced Demos Completed Successfully! ===")
    print("The factory pattern supports:")
    print("  ✓ Multi-robot setups with independent configs")
    print("  ✓ Comprehensive sensor ecosystems")
    print("  ✓ Flexible camera configurations")
    print("  ✓ Precision gripper settings")
    print("  ✓ Custom robot integration")
    print("  ✓ Priority-based configuration resolution")

if __name__ == "__main__":
    run_all_demos()
