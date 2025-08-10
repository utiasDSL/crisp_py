"""Unit tests for the Sensor classes and SensorConfig."""

from unittest.mock import Mock, patch

import numpy as np
import pytest
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from crisp_py.sensors.sensor import Float32ArraySensor, Sensor, TorqueSensor
from crisp_py.sensors.sensor_config import EmptySensorConfig, SensorConfig


class TestSensorConfig:
    """Test cases for the SensorConfig class."""

    def test_sensor_config_creation(self):
        """Test basic sensor config creation."""
        config = SensorConfig(shape=(3,))

        assert config.shape == (3,)
        assert config.name == "sensor"
        assert config.data_topic == "sensor_data"
        assert config.max_data_delay == 1.0

    def test_sensor_config_custom_values(self):
        """Test SensorConfig with custom values."""
        config = SensorConfig(
            shape=(6,),
            name="custom_sensor",
            data_topic="custom/data",
            max_data_delay=2.0,
        )

        assert config.shape == (6,)
        assert config.name == "custom_sensor"
        assert config.data_topic == "custom/data"
        assert config.max_data_delay == 2.0


class TestSensor:
    """Test cases for the abstract Sensor class."""

    def test_sensor_cannot_be_instantiated(self):
        """Test that Sensor abstract class cannot be instantiated."""
        with pytest.raises(TypeError):
            Sensor()

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_sensor_initialization_with_node(self, mock_callback_monitor, mock_rclpy):  # noqa: ANN001
        """Test sensor initialization with provided node."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True

        class ConcreteSensor(Sensor):
            def _create_subscription(self):
                pass

        sensor_config = EmptySensorConfig(name="sensor", shape=(3,))
        sensor = ConcreteSensor(sensor_config=sensor_config, node=mock_node, spin_node=False)

        assert sensor.node == mock_node
        assert sensor.config.name == "sensor"
        mock_callback_monitor.assert_called_once()

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_sensor_initialization_without_node(self, mock_callback_monitor, mock_rclpy):  # noqa: ANN001
        """Test sensor initialization without provided node."""
        mock_rclpy.ok.return_value = True
        mock_node = Mock()
        mock_rclpy.create_node.return_value = mock_node

        class ConcreteSensor(Sensor):
            def _create_subscription(self):
                pass

        sensor = ConcreteSensor(sensor_config=EmptySensorConfig(), node=None, spin_node=False)

        assert sensor.node == mock_node
        mock_rclpy.create_node.assert_called_once()

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_sensor_is_ready(self, mock_callback_monitor, mock_rclpy):  # noqa: ANN001
        """Test sensor is_ready method."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True

        class ConcreteSensor(Sensor):
            def _create_subscription(self):
                pass

        sensor = ConcreteSensor(sensor_config=EmptySensorConfig(), node=mock_node, spin_node=False)

        assert not sensor.is_ready()

        sensor._value = np.array([1.0, 2.0])
        assert sensor.is_ready()

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_sensor_value_property_not_ready(self, mock_callback_monitor, mock_rclpy):  # noqa: ANN001
        """Test sensor value property when not ready."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True

        class ConcreteSensor(Sensor):
            def _create_subscription(self):
                pass

        sensor = ConcreteSensor(sensor_config=EmptySensorConfig(), node=mock_node, spin_node=False)

        with pytest.raises(ValueError, match="Sensor value is not available yet"):
            _ = sensor.value

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_sensor_value_property_ready(self, mock_callback_monitor, mock_rclpy):  # noqa: ANN001
        """Test sensor value property when ready."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True
        mock_callback_monitor_instance = Mock()
        mock_callback_monitor.return_value = mock_callback_monitor_instance

        class ConcreteSensor(Sensor):
            def _create_subscription(self):
                pass

        sensor = ConcreteSensor(sensor_config=EmptySensorConfig(), node=mock_node, spin_node=False)
        sensor._value = np.array([3.0, 4.0])
        sensor._baseline = np.array([1.0, 2.0])

        result = sensor.value

        np.testing.assert_array_equal(result, np.array([2.0, 2.0]))
        mock_callback_monitor_instance.check_callback_health.assert_called_once()

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_sensor_calibrate_to_zero(self, mock_callback_monitor, mock_rclpy):  # noqa: ANN001
        """Test sensor calibrate_to_zero method."""
        mock_node = Mock()
        mock_rate = Mock()
        mock_node.create_rate.return_value = mock_rate
        mock_rclpy.ok.return_value = True
        mock_callback_monitor_instance = Mock()
        mock_callback_monitor.return_value = mock_callback_monitor_instance

        class ConcreteSensor(Sensor):
            def _create_subscription(self):
                pass

        sensor = ConcreteSensor(sensor_config=EmptySensorConfig(), node=mock_node, spin_node=False)
        sensor._value = np.array([1.0, 2.0])
        sensor._baseline = np.array([0.0, 0.0])

        sensor.calibrate_to_zero(num_samples=2, sample_rate=10.0)

        np.testing.assert_array_equal(sensor._baseline, np.array([1.0, 2.0]))
        assert mock_rate.sleep.call_count == 2


class TestFloat32ArraySensor:
    """Test cases for the Float32ArraySensor class."""

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_float32array_sensor_subscription(self, mock_callback_monitor, mock_rclpy):  # noqa: ANN001
        """Test Float32ArraySensor creates correct subscription."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True

        Float32ArraySensor(
            sensor_config=EmptySensorConfig(shape=(7,)), node=mock_node, spin_node=False
        )

        mock_node.create_subscription.assert_called_once()
        args, _ = mock_node.create_subscription.call_args
        assert args[0] == Float32MultiArray
        assert args[1] == "sensor_data"  # default topic

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_float32array_sensor_callback(self, mock_callback_monitor, mock_rclpy):  # noqa: ANN001
        """Test Float32ArraySensor callback processing."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True

        sensor = Float32ArraySensor(
            sensor_config=EmptySensorConfig(shape=(3,)), node=mock_node, spin_node=False
        )

        msg = Float32MultiArray()
        msg.data = [1.0, 2.0, 3.0]

        sensor._callback_sensor_data(msg)

        np.testing.assert_array_equal(sensor._value, np.array([1.0, 2.0, 3.0]))
        np.testing.assert_array_equal(sensor._baseline, np.array([0.0, 0.0, 0.0]))


class TestTorqueSensor:
    """Test cases for the TorqueSensor class."""

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_torque_sensor_subscription(self, mock_callback_monitor, mock_rclpy):  # noqa: ANN001
        """Test TorqueSensor creates correct subscription."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True
        sensor_config = EmptySensorConfig(shape=(3,))

        TorqueSensor(sensor_config=sensor_config, node=mock_node, spin_node=False)

        mock_node.create_subscription.assert_called_once()
        args, _ = mock_node.create_subscription.call_args
        assert args[0] == JointState
        assert args[1] == "sensor_data"  # default topic

    @patch("crisp_py.sensors.sensor.rclpy")
    @patch("crisp_py.sensors.sensor.CallbackMonitor")
    def test_torque_sensor_callback(self, mock_callback_monitor, mock_rclpy):  # noqa: ANN001
        """Test TorqueSensor callback processing."""
        mock_node = Mock()
        mock_rclpy.ok.return_value = True

        sentor_config = EmptySensorConfig(shape=(3,))
        sensor = TorqueSensor(sensor_config=sentor_config, node=mock_node, spin_node=False)

        msg = JointState()
        msg.effort = [0.5, 1.0, 1.5]

        sensor._callback_joint_state(msg)

        np.testing.assert_array_equal(sensor._value, np.array([0.5, 1.0, 1.5]))
        np.testing.assert_array_equal(sensor._baseline, np.array([0.0, 0.0, 0.0]))
