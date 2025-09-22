"""Factory pattern implementation for creating CRISP components with configurable priority system."""

import logging
from pathlib import Path
from typing import Any, Dict, Literal, Optional

import yaml
from rclpy.node import Node

from crisp_py.camera.camera import Camera
from crisp_py.camera.camera_config import CameraConfig
from crisp_py.constants import (
    COMPONENT_TYPES,
    CUSTOM_ROBOT_TYPES,
    CUSTOM_SENSOR_TYPES,
)
from crisp_py.gripper.gripper import Gripper, GripperConfig
from crisp_py.robot import Robot
from crisp_py.robot_config import RobotConfig, make_robot_config
from crisp_py.sensors.sensor import make_sensor
from crisp_py.sensors.sensor_config import SensorConfig, make_sensor_config
from crisp_py.types import (
    ComponentType,
    CustomCameraType,
    CustomRobotType,
    CustomSensorType,
)
from crisp_py.utils.config import find_config


class CrispFactory:
    """Factory for creating CRISP components with configurable priority system.

    The CrispFactory provides a centralized way to create and configure CRISP components
    (robots, cameras, grippers, sensors) with flexible configuration management.

    ## Configuration Priority System

    The factory resolves configurations in the following priority order:
    1. **Programmatically registered configs** (highest priority)
    2. **YAML configuration files** (found in CRISP_CONFIG_PATH)
    3. **Default configurations** (lowest priority)

    ## Supported Component Types

    - **robots**: franka, kinova, iiwa, so101, custom, generic
    - **cameras**: default, custom
    - **grippers**: default, custom
    - **sensors**: empty, float32, force_torque, custom

    ## Usage Patterns

    ### Basic Usage with Defaults
    ```python
    factory = CrispFactory()
    robot = factory.create_robot(robot_type="franka")
    camera = factory.create_camera()
    gripper = factory.create_gripper()
    ```

    ### Programmatic Configuration
    ```python
    factory = CrispFactory()

    # Register custom configuration
    custom_config = FrankaConfig(home_config=[0, -0.5, 0, -2.5, 0, 2.0, 0.8], time_to_home=3.0)
    factory.register_config(
        component_type="robot",
        type_name="franka",
        config_name="lab_setup",
        config_instance=custom_config,
    )

    # Use registered configuration
    robot = factory.create_robot(robot_type="franka", config_name="lab_setup")
    ```

    ### YAML Configuration
    ```python
    # Create config file: my_robot.yaml
    # home_config: [0, -0.5, 0, -2.5, 0, 2.0, 0.8]
    # time_to_home: 3.0

    factory = CrispFactory()
    robot = factory.create_robot(
        robot_type="franka",
        config_name="my_robot",  # Loads my_robot.yaml
    )
    ```

    ### Custom Robot Types
    ```python
    factory = CrispFactory()

    # Custom robots require programmatic or YAML config
    custom_robot_config = RobotConfig(
        joint_names=["j1", "j2", "j3", "j4"],
        home_config=[0.0, 0.0, 0.0, 0.0],
        base_frame="custom_base",
        target_frame="custom_ee",
    )

    factory.register_config(
        component_type="robot",
        type_name="my_robot_type",
        config_name="lab_config",
        config_instance=custom_robot_config,
    )

    robot = factory.create_robot(robot_type="my_robot_type", config_name="lab_config")
    ```

    ### Multi-Robot Setup
    ```python
    factory = CrispFactory()

    # Register configs for different arms
    factory.register_config(
        component_type="robot",
        type_name="franka",
        config_name="left_arm",
        config_instance=left_arm_config,
    )

    factory.register_config(
        component_type="robot",
        type_name="franka",
        config_name="right_arm",
        config_instance=right_arm_config,
    )

    # Create robots with different configs
    left_robot = factory.create_robot(robot_type="franka", config_name="left_arm", namespace="left")

    right_robot = factory.create_robot(
        robot_type="franka", config_name="right_arm", namespace="right"
    )
    ```

    ## Thread Safety

    Each CrispFactory instance is thread-safe for read operations after initial
    configuration. For write operations (register_config), external synchronization
    may be needed in multi-threaded environments.

    ## Best Practices

    1. **Create one factory per logical setup** (e.g., simulation vs real robot)
    2. **Register all configs early** in your application lifecycle
    3. **Use descriptive config_name values** for easier debugging
    4. **Leverage the priority system**: programmatic for dynamic configs,
       YAML for static configs, defaults for prototyping
    5. **Use custom types for user-defined robots/sensors** that don't fit
       predefined categories

    All creation methods use keyword-only arguments for clarity and type safety.
    """

    def __init__(self):
        """Initialize the factory with empty programmatic config registry."""
        self._programmatic_configs: Dict[str, Any] = {}
        self._logger = logging.getLogger(__name__)

    def register_config(
        self,
        *,
        component_type: ComponentType,
        type_name: str,
        config_name: str,
        config_instance: Any,
    ) -> None:
        """Register a programmatic configuration (highest priority).

        Programmatically registered configurations take the highest priority in the
        configuration resolution system, overriding both YAML files and defaults.

        Args:
            component_type: Component type ("robot", "camera", "gripper", "sensor")
            type_name: Specific type name (e.g., "franka", "kinova", "default", or custom types)
            config_name: Custom name for this config variant (e.g., "lab_setup", "production")
            config_instance: The actual config object (RobotConfig, CameraConfig, etc.)

        Example:
            ```python
            factory = CrispFactory()

            # Register a custom Franka configuration
            lab_config = FrankaConfig(home_config=[0, -0.5, 0, -2.5, 0, 2.0, 0.8], time_to_home=3.0)
            factory.register_config(
                component_type="robot",
                type_name="franka",
                config_name="lab_setup",
                config_instance=lab_config,
            )
            ```

        Raises:
            ValueError: If component_type is not supported
            TypeError: If config_instance is not the expected type for component_type
        """
        # Validate component type
        if component_type not in COMPONENT_TYPES:
            raise ValueError(
                f"Invalid component_type '{component_type}'. Must be one of: {COMPONENT_TYPES}"
            )

        self._validate_config_instance(component_type, config_instance)

        key = f"{component_type}_{type_name}_{config_name}"
        self._programmatic_configs[key] = config_instance
        self._logger.info(f"Registered programmatic config: {key}")

    def create_robot(
        self,
        *,
        robot_type: CustomRobotType,
        config_name: Optional[str] = None,
        node: Optional[Node] = None,
        **overrides,  # noqa: ANN003
    ) -> Robot:
        """Create a robot instance with automatic configuration resolution.

        Creates a Robot instance using the factory's configuration priority system.
        Supports both predefined robot types (franka, kinova, iiwa, so101) and
        custom robot types.

        Args:
            robot_type: Type of robot. Predefined: "franka", "kinova", "iiwa", "so101".
                       Custom: "custom", "generic", or any user-defined string.
            config_name: Optional name of registered config or YAML file (without .yaml).
                        If None, uses default configuration.
            node: Optional ROS2 node to use. If None, creates a new node.
            **overrides: Additional keyword arguments passed to Robot constructor
                        (e.g., namespace="left_arm")

        Returns:
            Robot: Configured robot instance ready for use

        Example:
            ```python
            factory = CrispFactory()

            # Using defaults
            robot = factory.create_robot(robot_type="franka")

            # Using registered config
            robot = factory.create_robot(
                robot_type="franka", config_name="lab_setup", namespace="left_arm"
            )

            # Custom robot type (requires registered config)
            robot = factory.create_robot(robot_type="my_custom_robot", config_name="lab_config")
            ```

        Raises:
            ValueError: If custom robot_type is used without registered config or YAML
        """
        config = self._resolve_config("robot", robot_type, config_name)
        return Robot(robot_config=config, node=node, **overrides)

    def create_camera(
        self,
        *,
        camera_type: CustomCameraType = "default",
        config_name: Optional[str] = None,
        node: Optional[Node] = None,
        **overrides,  # noqa: ANN003
    ) -> Camera:
        """Create a camera instance with automatic configuration resolution.

        Args:
            camera_type: Type of camera. Use "default" for standard cameras or
                        "custom" for user-defined camera types.
            config_name: Optional name of registered config or YAML file (without .yaml).
                        If None, uses default configuration.
            node: Optional ROS2 node to use. If None, creates a new node.
            **overrides: Additional keyword arguments passed to Camera constructor
                        (e.g., namespace="overhead")

        Returns:
            Camera: Configured camera instance ready for use

        Example:
            ```python
            factory = CrispFactory()

            # Using defaults
            camera = factory.create_camera()

            # Using registered config
            camera = factory.create_camera(config_name="overhead_hd", namespace="overhead")
            ```
        """
        config = self._resolve_config("camera", camera_type, config_name)
        return Camera(config=config, node=node, **overrides)

    def create_gripper(
        self,
        *,
        gripper_type: Literal["default", "hello"] = "default",
        config_name: Optional[str] = None,
        node: Optional[Node] = None,
        **overrides,  # noqa: ANN003
    ) -> Gripper:
        """Create a gripper instance with automatic configuration resolution.

        Args:
            gripper_type: Type of gripper. Use "default" for standard grippers or
                         "custom" for user-defined gripper types.
            config_name: Optional name of registered config or YAML file (without .yaml).
                        If None, uses default configuration.
            node: Optional ROS2 node to use. If None, creates a new node.
            **overrides: Additional keyword arguments passed to Gripper constructor
                        (e.g., namespace="left")

        Returns:
            Gripper: Configured gripper instance ready for use

        Example:
            ```python
            factory = CrispFactory()

            # Using defaults
            gripper = factory.create_gripper()

            # Using registered precision config
            gripper = factory.create_gripper(config_name="micro_precision", namespace="left")
            ```
        """
        config = self._resolve_config("gripper", gripper_type, config_name)
        return Gripper(gripper_config=config, node=node, **overrides)

    def create_sensor(
        self,
        *,
        sensor_type: CustomSensorType,
        config_name: Optional[str] = None,
        node: Optional[Node] = None,
        **overrides,
    ) -> Any:
        """Create a sensor instance with automatic configuration resolution.

        Args:
            sensor_type: Type of sensor. Predefined: "empty", "float32", "force_torque".
                        Custom: "custom" or any user-defined string.
            config_name: Optional name of registered config or YAML file (without .yaml).
                        If None, uses default configuration (unavailable for custom types).
            node: Optional ROS2 node to use. If None, creates a new node.
            **overrides: Additional keyword arguments passed to sensor constructor

        Returns:
            Sensor: Configured sensor instance ready for use

        Example:
            ```python
            factory = CrispFactory()

            # Force-torque sensor with defaults
            ft_sensor = factory.create_sensor(sensor_type="force_torque")

            # Custom sensor with registered config
            tactile = factory.create_sensor(sensor_type="custom", config_name="fingertip_tactile")
            ```

        Raises:
            ValueError: If custom sensor_type is used without registered config or YAML
        """
        config = self._resolve_config("sensor", sensor_type, config_name)
        return make_sensor(sensor_config=config, node=node, **overrides)

    def _resolve_config(
        self, component_type: str, type_name: str, config_name: Optional[str]
    ) -> Any:
        """Resolve configuration using priority system.

        Priority order:
        1. Programmatically registered configs
        2. YAML configuration files
        3. Default configurations

        Args:
            component_type: Type of component
            type_name: Specific type name (supports custom types)
            config_name: Optional config name

        Returns:
            Configuration object
        """
        config_source = None
        config = None

        # Priority 1: Programmatic registration
        if config_name:
            key = f"{component_type}_{type_name}_{config_name}"
            if key in self._programmatic_configs:
                config = self._programmatic_configs[key]
                config_source = f"programmatic: {key}"

        # Priority 2: YAML file
        if config is None and config_name:
            yaml_path = find_config(f"{config_name}.yaml")
            if yaml_path:
                config = self._load_yaml_config(yaml_path, component_type, type_name)
                config_source = f"yaml_file: {yaml_path}"

        # Priority 3: Default configuration
        if config is None:
            config = self._get_default_config(component_type, type_name)
            config_source = f"default for {component_type}:{type_name}"

        self._logger.info(f"Using config from {config_source}")
        self._validate_config_instance(component_type, config)
        return config

    def _load_yaml_config(self, yaml_path: Path, component_type: str, type_name: str) -> Any:
        """Load configuration from YAML file.

        Args:
            yaml_path: Path to YAML configuration file
            component_type: Type of component
            type_name: Specific type name (supports custom types)

        Returns:
            Configuration object

        Raises:
            ValueError: If YAML file is invalid or missing required fields
        """
        try:
            with open(yaml_path, "r") as file:
                yaml_data = yaml.safe_load(file)

            if component_type == "robot":
                # For custom/generic robots, create a base RobotConfig with YAML data
                if type_name in CUSTOM_ROBOT_TYPES:
                    return RobotConfig(**yaml_data)
                else:
                    return make_robot_config(type_name, **yaml_data)
            elif component_type == "camera":
                return CameraConfig(**yaml_data)
            elif component_type == "gripper":
                return GripperConfig(**yaml_data)
            elif component_type == "sensor":
                # For custom sensors, try to create base SensorConfig if type not recognized
                try:
                    return make_sensor_config(type_name, **yaml_data)
                except ValueError:
                    # If type_name not recognized, create base SensorConfig
                    return SensorConfig(**yaml_data)
            else:
                raise ValueError(f"Unknown component type: {component_type}")

        except Exception as e:
            raise ValueError(f"Failed to load YAML config from {yaml_path}: {e}")

    def _get_default_config(self, component_type: str, type_name: str) -> Any:
        """Get default configuration for component type.

        Args:
            component_type: Type of component
            type_name: Specific type name (supports custom types)

        Returns:
            Default configuration object

        Raises:
            ValueError: If component type is not supported or required config is missing
        """
        if component_type == "robot":
            # For custom/generic robots, require programmatic registration or YAML config
            if type_name in CUSTOM_ROBOT_TYPES:
                raise ValueError(
                    f"Custom robot type '{type_name}' requires programmatic registration "
                    "or YAML configuration. No default config available."
                )
            else:
                return make_robot_config(type_name)
        elif component_type == "camera":
            return CameraConfig()
        elif component_type == "gripper":
            return GripperConfig(min_value=0.0, max_value=1.0)
        elif component_type == "sensor":
            # For custom sensors, require programmatic registration or YAML config
            if type_name in CUSTOM_SENSOR_TYPES:
                raise ValueError(
                    f"Custom sensor type '{type_name}' requires programmatic registration "
                    "or YAML configuration. No default config available."
                )
            else:
                return make_sensor_config(type_name)
        else:
            raise ValueError(f"Unknown component type: {component_type}")

    def _validate_config_instance(self, component_type: str, config_instance: Any) -> None:
        """Validate that config instance matches expected type.

        Args:
            component_type: Type of component
            config_instance: Configuration instance to validate

        Raises:
            TypeError: If config instance is not of expected type
        """
        expected_types = {
            "robot": RobotConfig,
            "camera": CameraConfig,
            "gripper": GripperConfig,
            "sensor": SensorConfig,
        }

        expected_type = expected_types.get(component_type)
        if expected_type and not isinstance(config_instance, expected_type):
            raise TypeError(
                f"Expected {expected_type.__name__} for {component_type}, "
                f"got {type(config_instance).__name__}"
            )

        self._logger.debug(
            f"Validated config instance for {component_type}: {type(config_instance).__name__}"
        )
