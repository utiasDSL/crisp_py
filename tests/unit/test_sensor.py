"""Unit tests for the Sensor classes and SensorConfig."""

from typing import Any
from unittest.mock import Mock, mock_open, patch

import numpy as np
import pytest
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray

from crisp_py.sensors import (
    Sensor,
    SensorConfig,
    get_float32_array_sensor_spec,
    get_force_torque_sensor_spec,
    get_sensor_spec,
    list_sensor_configs,
    make_sensor,
    register_sensor,
    sensor_registry,
)


class TestSensorConfig:
    """Test cases for the SensorConfig class."""

    def test_sensor_config_creation(self) -> None:
        """Test basic sensor config creation."""
        config = SensorConfig(
            name="test_sensor", 
            shape=(3, 3), 
            sensor_type="float32_array"
        )
        assert config.name == "test_sensor"
        assert config.shape == (3, 3)
        assert config.sensor_type == "float32_array"
        assert config.data_topic == "sensor_data"  # default
        assert config.max_data_delay == 1.0  # default

    def test_sensor_config_custom_values(self) -> None:
        """Test sensor config with custom values."""
        config = SensorConfig(
            name="custom_sensor",
            shape=(2, 2),
            data_topic="custom_topic",
            max_data_delay=2.0,
            sensor_type="force_torque",
            buffer_size=100,
        )
        assert config.name == "custom_sensor"
        assert config.shape == (2, 2)
        assert config.data_topic == "custom_topic"
        assert config.max_data_delay == 2.0
        assert config.sensor_type == "force_torque"
        assert config.buffer_size == 100

    def test_sensor_config_from_yaml(self) -> None:
        """Test loading sensor config from YAML."""
        yaml_data = """
name: yaml_sensor
shape: [6]
sensor_type: force_torque
data_topic: wrench_topic
max_data_delay: 0.5
"""
        with patch("builtins.open", mock_open(read_data=yaml_data)):
            config = SensorConfig.from_yaml("test.yaml")
            assert config.name == "yaml_sensor"
            assert config.shape == [6]  # YAML loads as list, not tuple
            assert config.sensor_type == "force_torque"
            assert config.data_topic == "wrench_topic"
            assert config.max_data_delay == 0.5


class TestSensorRegistry:
    """Test cases for sensor registration and spec retrieval."""

    def test_register_sensor_decorator(self) -> None:
        """Test registering a sensor type."""
        @register_sensor("test_sensor")
        def test_sensor_spec() -> tuple[type, Any]:
            return (Float32MultiArray, lambda msg: np.array([1.0, 2.0]))
        
        assert "test_sensor" in sensor_registry
        assert sensor_registry["test_sensor"] == test_sensor_spec

    def test_get_sensor_spec(self) -> None:
        """Test retrieving sensor spec."""
        # Register a test sensor
        @register_sensor("test_spec")
        def test_spec() -> tuple[type, Any]:
            return (Float32MultiArray, lambda msg: np.array([1.0]))
        
        msg_type, conversion_func = get_sensor_spec("test_spec")
        assert msg_type == Float32MultiArray
        
        # Test conversion function
        msg = Float32MultiArray()
        msg.data = [5.0]
        result = conversion_func(msg)
        assert np.array_equal(result, np.array([1.0]))

    def test_get_sensor_spec_unknown_type(self) -> None:
        """Test error when getting unknown sensor type."""
        with pytest.raises(ValueError, match="Unknown sensor type: unknown"):
            get_sensor_spec("unknown")


class TestSensorSpecs:
    """Test cases for built-in sensor specifications."""

    def test_float32_array_sensor_spec(self) -> None:
        """Test Float32Array sensor specification."""
        msg_type, conversion_func = get_float32_array_sensor_spec()
        
        assert msg_type == Float32MultiArray
        
        # Test conversion function
        msg = Float32MultiArray()
        msg.data = [1.0, 2.0, 3.0]
        result = conversion_func(msg)
        
        np.testing.assert_array_equal(result, np.array([1.0, 2.0, 3.0], dtype=np.float32))

    def test_force_torque_sensor_spec(self) -> None:
        """Test force-torque sensor specification."""
        msg_type, conversion_func = get_force_torque_sensor_spec()
        
        assert msg_type == WrenchStamped
        
        # Test conversion function
        msg = WrenchStamped()
        msg.wrench.force.x = 0.5
        msg.wrench.force.y = 1.0
        msg.wrench.force.z = 1.5
        msg.wrench.torque.x = 0.1
        msg.wrench.torque.y = 0.2
        msg.wrench.torque.z = 0.3
        
        result = conversion_func(msg)
        expected = np.array([0.5, 1.0, 1.5, 0.1, 0.2, 0.3], dtype=np.float32)
        
        np.testing.assert_array_equal(result, expected)


class TestSensor:
    """Test cases for the Sensor class."""

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_sensor_initialization_with_node(self, mock_callback_monitor: Any, mock_rclpy: Any) -> None:
        """Test sensor initialization with provided node."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True
        mock_node.create_subscription.return_value = Mock()
        mock_node.create_client.return_value = Mock()

        sensor_config = SensorConfig(
            name="test_sensor",
            shape=(3,),
            sensor_type="float32_array"
        )
        sensor = Sensor(sensor_config=sensor_config, node=mock_node, spin_node=False)

        assert sensor.node == mock_node
        assert sensor.config.name == "test_sensor"
        mock_callback_monitor.assert_called_once()
        mock_node.create_subscription.assert_called_once()

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_sensor_initialization_without_node(self, mock_callback_monitor: Any, mock_rclpy: Any) -> None:
        """Test sensor initialization without provided node."""
        mock_rclpy.ok.return_value = True
        mock_node = Mock()
        mock_rclpy.create_node.return_value = mock_node
        mock_node.create_subscription.return_value = Mock()
        mock_node.create_client.return_value = Mock()

        sensor_config = SensorConfig(shape=(3,), sensor_type="float32_array")
        sensor = Sensor(sensor_config=sensor_config, node=None, spin_node=False)

        assert sensor.node == mock_node
        mock_rclpy.create_node.assert_called_once()

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_sensor_is_ready(self, mock_callback_monitor: Any, mock_rclpy: Any) -> None:
        """Test sensor is_ready method."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True
        mock_node.create_subscription.return_value = Mock()
        mock_node.create_client.return_value = Mock()

        sensor_config = SensorConfig(shape=(3,), sensor_type="float32_array")
        sensor = Sensor(sensor_config=sensor_config, node=mock_node, spin_node=False)

        # Initially not ready
        assert not sensor.is_ready()

        # Set value to make it ready
        sensor._value = np.array([1.0, 2.0, 3.0])
        assert sensor.is_ready()

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_sensor_value_property(self, mock_callback_monitor: Any, mock_rclpy: Any) -> None:
        """Test sensor value property."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True
        mock_node.create_subscription.return_value = Mock()
        mock_node.create_client.return_value = Mock()

        sensor_config = SensorConfig(shape=(3,), sensor_type="float32_array")
        sensor = Sensor(sensor_config=sensor_config, node=mock_node, spin_node=False)

        # Test error when not ready
        with pytest.raises(ValueError, match="Sensor value is not available yet"):
            _ = sensor.value

        # Test value when ready
        sensor._value = np.array([3.0, 4.0, 5.0])

        result = sensor.value
        np.testing.assert_array_equal(result, np.array([3.0, 4.0, 5.0]))

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_sensor_callback(self, mock_callback_monitor: Any, mock_rclpy: Any) -> None:
        """Test sensor callback processing."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True
        mock_node.create_subscription.return_value = Mock()
        mock_node.create_client.return_value = Mock()

        sensor_config = SensorConfig(shape=(3,), sensor_type="float32_array")
        sensor = Sensor(sensor_config=sensor_config, node=mock_node, spin_node=False)

        # Simulate callback
        msg = Float32MultiArray()
        msg.data = [1.0, 2.0, 3.0]

        sensor._sensor_callback(msg)

        np.testing.assert_array_equal(sensor._value, np.array([1.0, 2.0, 3.0]))

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_sensor_buffer_property(self, mock_callback_monitor: Any, mock_rclpy: Any) -> None:
        """Test sensor buffer property."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True
        mock_node.create_subscription.return_value = Mock()
        mock_node.create_client.return_value = Mock()

        sensor_config = SensorConfig(shape=(2,), sensor_type="float32_array", buffer_size=5)
        sensor = Sensor(sensor_config=sensor_config, node=mock_node, spin_node=False)

        # Test buffer property
        buffer = sensor.buffer
        assert buffer is not None
        assert hasattr(buffer, 'add')  # SlidingBuffer should have add method


class TestMakeSensor:
    """Test cases for the make_sensor factory function."""

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_make_sensor_from_yaml(self, mock_callback_monitor: Any, mock_rclpy: Any) -> None:
        """Test make_sensor from YAML config."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True
        mock_node.create_subscription.return_value = Mock()
        mock_node.create_client.return_value = Mock()

        yaml_data = """
name: test_sensor
shape: [3]
sensor_type: float32_array
data_topic: test_topic
"""

        with patch("crisp_py.sensors.sensor.find_config") as mock_find_config:
            with patch("builtins.open", mock_open(read_data=yaml_data)):
                mock_find_config.return_value = "/path/to/test_config.yaml"

                sensor = make_sensor(config_name="test_config", node=mock_node, spin_node=False)

                assert isinstance(sensor, Sensor)
                assert sensor.config.name == "test_sensor"
                assert sensor.config.sensor_type == "float32_array"
                assert sensor.config.data_topic == "test_topic"

    @patch("crisp_py.sensors.sensor.find_config")
    def test_make_sensor_file_not_found(self, mock_find_config: Any) -> None:
        """Test make_sensor with missing config file."""
        mock_find_config.return_value = None
        
        with pytest.raises(FileNotFoundError, match="Sensor config file 'missing.yaml' not found"):
            make_sensor(config_name="missing")

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_make_sensor_with_overrides(self, mock_callback_monitor: Any, mock_rclpy: Any) -> None:
        """Test make_sensor with parameter overrides."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True
        mock_node.create_subscription.return_value = Mock()
        mock_node.create_client.return_value = Mock()

        yaml_data = """
name: base_sensor
shape: [3]
sensor_type: float32_array
"""

        with patch("crisp_py.sensors.sensor.find_config") as mock_find_config:
            with patch("builtins.open", mock_open(read_data=yaml_data)):
                mock_find_config.return_value = "/path/to/config.yaml"

                sensor = make_sensor(
                    config_name="config",
                    node=mock_node,
                    spin_node=False,
                    name="overridden_sensor",
                    data_topic="overridden_topic"
                )

                assert sensor.config.name == "overridden_sensor"
                assert sensor.config.data_topic == "overridden_topic"


class TestUtilityFunctions:
    """Test cases for utility functions."""

    @patch("crisp_py.sensors.sensor.list_configs_in_folder")
    def test_list_sensor_configs(self, mock_list_configs: Any) -> None:
        """Test listing sensor configurations."""
        from pathlib import Path
        
        mock_configs = [
            Path("sensor1.yaml"),
            Path("sensor2.yaml"),
            Path("not_yaml.txt"),
        ]
        mock_list_configs.return_value = mock_configs
        
        configs = list_sensor_configs()
        
        assert configs == ["sensor1", "sensor2"]
        mock_list_configs.assert_called_once_with("sensors")