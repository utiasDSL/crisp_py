"""Unit tests for the Gripper class and GripperConfig."""

from pathlib import Path
from unittest.mock import Mock, mock_open, patch

import pytest

from crisp_py.gripper.gripper import Gripper, GripperConfig


class TestGripperConfig:
    """Test cases for the GripperConfig class."""

    def test_gripper_config_creation(self):
        """Test basic gripper config creation."""
        config = GripperConfig(min_value=0.0, max_value=1.0)

        assert config.min_value == 0.0
        assert config.max_value == 1.0
        assert config.command_topic == "gripper_position_controller/commands"
        assert config.joint_state_topic == "joint_states"
        assert config.reboot_service == "reboot_gripper"
        assert config.enable_torque_service == "dynamixel_hardware_interface/set_dxl_torque"
        assert config.index == 0
        assert config.publish_frequency == 30.0
        assert config.max_joint_delay == 1.0
        assert config.max_delta == 0.1

    def test_gripper_config_custom_values(self):
        """Test GripperConfig with custom values."""
        config = GripperConfig(
            min_value=0.1,
            max_value=0.9,
            command_topic="custom/commands",
            joint_state_topic="custom/joint_states",
            reboot_service="custom/reboot",
            enable_torque_service="custom/torque",
            index=1,
            publish_frequency=50.0,
            max_joint_delay=2.0,
            max_delta=0.2,
        )

        assert config.min_value == 0.1
        assert config.max_value == 0.9
        assert config.command_topic == "custom/commands"
        assert config.joint_state_topic == "custom/joint_states"
        assert config.reboot_service == "custom/reboot"
        assert config.enable_torque_service == "custom/torque"
        assert config.index == 1
        assert config.publish_frequency == 50.0
        assert config.max_joint_delay == 2.0
        assert config.max_delta == 0.2

    def test_gripper_config_from_yaml_string_path(self):
        """Test GripperConfig.from_yaml with string path."""
        yaml_content = """
min_value: 0.1
max_value: 0.9
command_topic: "custom_commands"
joint_state_topic: "custom_joint_states"
reboot_service: "custom_reboot"
enable_torque_service: "custom_torque"
index: 2
publish_frequency: 60.0
max_delta: 0.3
max_joint_delay: 1.5
"""

        with patch("builtins.open", mock_open(read_data=yaml_content)):
            with patch("pathlib.Path.parent") as mock_parent:
                mock_parent.parent.parent = Path("/fake/project/root")
                config = GripperConfig.from_yaml("config/gripper.yaml")

        assert config.min_value == 0.1
        assert config.max_value == 0.9
        assert config.command_topic == "custom_commands"
        assert config.joint_state_topic == "custom_joint_states"
        assert config.reboot_service == "custom_reboot"
        assert config.enable_torque_service == "custom_torque"
        assert config.index == 2
        assert config.publish_frequency == 60.0
        assert config.max_delta == 0.3
        assert config.max_joint_delay == 1.5

    def test_gripper_config_from_yaml_path_object(self):
        """Test GripperConfig.from_yaml with Path object."""
        yaml_content = """
min_value: 0.2
max_value: 0.8
"""

        with patch("builtins.open", mock_open(read_data=yaml_content)):
            config = GripperConfig.from_yaml(Path("/fake/path/config.yaml"))

        assert config.min_value == 0.2
        assert config.max_value == 0.8
        # Check defaults are applied
        assert config.command_topic == "gripper_position_controller/commands"
        assert config.index == 0

    def test_gripper_config_from_yaml_partial_config(self):
        """Test GripperConfig.from_yaml with partial configuration."""
        yaml_content = """
min_value: 0.3
max_value: 0.7
command_topic: "partial_commands"
"""

        with patch("builtins.open", mock_open(read_data=yaml_content)):
            config = GripperConfig.from_yaml(Path("/fake/path/config.yaml"))

        assert config.min_value == 0.3
        assert config.max_value == 0.7
        assert config.command_topic == "partial_commands"
        # Check defaults are applied for missing values
        assert config.joint_state_topic == "joint_states"
        assert config.reboot_service == "reboot_gripper"
        assert config.index == 0

    def test_gripper_config_from_yaml_invalid_path_type(self):
        """Test GripperConfig.from_yaml with invalid path type."""
        with pytest.raises(TypeError, match="Path must be a string or a Path object"):
            GripperConfig.from_yaml(123)

    def test_gripper_config_from_yaml_empty_file(self):
        """Test GripperConfig.from_yaml with empty YAML file."""
        with patch("builtins.open", mock_open(read_data="")):
            with patch("yaml.safe_load", return_value={}):
                config = GripperConfig.from_yaml(Path("/fake/path/config.yaml"))

        # All should be defaults
        assert config.min_value == 0.0
        assert config.max_value == 1.0
        assert config.command_topic == "gripper_position_controller/commands"
        assert config.index == 0


class TestGripperBasics:
    """Test basic Gripper class functionality."""

    def test_gripper_threads_required(self):
        """Test that Gripper class has correct thread requirements."""
        assert Gripper.THREADS_REQUIRED == 2

    def test_gripper_init_with_custom_config(self):
        """Test gripper initialization with custom config."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                custom_config = GripperConfig(min_value=0.1, max_value=0.9)
                gripper = Gripper(gripper_config=custom_config, spin_node=False)

                assert gripper.config == custom_config
                assert gripper.min_value == 0.1
                assert gripper.max_value == 0.9

    def test_gripper_init_with_default_config(self):
        """Test gripper initialization with default config."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(spin_node=False)

                assert isinstance(gripper.config, GripperConfig)
                assert gripper.min_value == 0.0
                assert gripper.max_value == 1.0

    def test_gripper_init_with_namespace(self):
        """Test gripper initialization with namespace."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(namespace="test_gripper", spin_node=False)

                assert gripper._prefix == "test_gripper_"

    def test_gripper_init_without_namespace(self):
        """Test gripper initialization without namespace."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(spin_node=False)

                assert gripper._prefix == ""


class TestGripperProperties:
    """Test gripper property access and behavior."""

    def test_gripper_properties_initial_values(self):
        """Test gripper properties have correct initial values."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(spin_node=False)

                assert gripper.torque is None
                assert gripper.raw_value is None
                assert not gripper.is_ready()

    def test_gripper_value_error_when_not_initialized(self):
        """Test value property raises error when not initialized."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(spin_node=False)

                with pytest.raises(RuntimeError, match="Gripper is not initialized"):
                    gripper.value

    def test_gripper_value_returns_normalized_value(self):
        """Test value property returns normalized value."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = GripperConfig(min_value=0.0, max_value=2.0)
                gripper = Gripper(gripper_config=config, spin_node=False)

                # Mock freshness checker
                gripper._joint_freshness_checker = Mock()

                # Set raw value
                gripper._value = 1.0  # Should normalize to 0.5

                assert gripper.value == 0.5

    def test_gripper_value_clipped_to_valid_range(self):
        """Test value property clips to valid range."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = GripperConfig(min_value=0.0, max_value=1.0)
                gripper = Gripper(gripper_config=config, spin_node=False)

                # Mock freshness checker
                gripper._joint_freshness_checker = Mock()

                # Test clipping above max
                gripper._value = 1.5  # Should clip to 1.0
                assert gripper.value == 1.0

                # Test clipping below min
                gripper._value = -0.5  # Should clip to 0.0
                assert gripper.value == 0.0

    def test_gripper_is_ready_true_when_initialized(self):
        """Test is_ready returns True when gripper is initialized."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(spin_node=False)

                gripper._value = 0.5
                assert gripper.is_ready()

    def test_gripper_is_valid_checks_bounds(self):
        """Test is_valid checks if gripper value is within bounds."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = GripperConfig(min_value=0.0, max_value=1.0)
                gripper = Gripper(gripper_config=config, spin_node=False)

                # Mock logger
                gripper.node.get_logger.return_value = Mock()

                # Test uninitialized
                assert not gripper.is_valid

                # Test valid value
                gripper._value = 0.5
                assert gripper.is_valid

                # Test out of bounds value
                gripper._value = 2.0  # Normalizes to 2.0, which is > 1.05
                assert not gripper.is_valid


class TestGripperMethods:
    """Test gripper methods and functionality."""

    def test_gripper_open_sets_target_to_one(self):
        """Test open method sets target to 1.0."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = GripperConfig(min_value=0.0, max_value=2.0)
                gripper = Gripper(gripper_config=config, spin_node=False)

                gripper.open()

                assert gripper._target == 2.0  # unnormalized value of 1.0

    def test_gripper_close_sets_target_to_zero(self):
        """Test close method sets target to 0.0."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = GripperConfig(min_value=0.0, max_value=2.0)
                gripper = Gripper(gripper_config=config, spin_node=False)

                gripper.close()

                assert gripper._target == 0.0  # unnormalized value of 0.0

    def test_gripper_set_target_valid_range(self):
        """Test set_target with valid range."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = GripperConfig(min_value=0.0, max_value=2.0)
                gripper = Gripper(gripper_config=config, spin_node=False)

                gripper.set_target(0.5)

                assert gripper._target == 1.0  # unnormalized value of 0.5

    def test_gripper_set_target_with_epsilon(self):
        """Test set_target with epsilon tolerance."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = GripperConfig(min_value=0.0, max_value=2.0)
                gripper = Gripper(gripper_config=config, spin_node=False)

                # Should work with epsilon tolerance
                gripper.set_target(1.05, epsilon=0.1)

                assert gripper._target == 2.1  # unnormalized value of 1.05

    def test_gripper_set_target_out_of_range(self):
        """Test set_target raises error when out of range."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(spin_node=False)

                with pytest.raises(
                    AssertionError, match="The target should be normalized between 0 and 1"
                ):
                    gripper.set_target(1.5)

    def test_gripper_is_open_true_when_above_threshold(self):
        """Test is_open returns True when gripper is above threshold."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(spin_node=False)

                # Mock freshness checker
                gripper._joint_freshness_checker = Mock()

                gripper._value = 0.5  # normalized value is 0.5
                assert gripper.is_open(open_threshold=0.1)
                assert not gripper.is_open(open_threshold=0.8)

    def test_gripper_is_open_error_when_not_initialized(self):
        """Test is_open raises error when gripper not initialized."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(spin_node=False)

                with pytest.raises(RuntimeError, match="Gripper is not initialized"):
                    gripper.is_open()


class TestGripperNormalization:
    """Test gripper normalization and unnormalization functions."""

    def test_gripper_normalize_function(self):
        """Test _normalize function works correctly."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = GripperConfig(min_value=2.0, max_value=8.0)
                gripper = Gripper(gripper_config=config, spin_node=False)

                # Test normalization
                assert gripper._normalize(2.0) == 0.0  # min_value
                assert gripper._normalize(8.0) == 1.0  # max_value
                assert gripper._normalize(5.0) == 0.5  # middle value

    def test_gripper_unnormalize_function(self):
        """Test _unnormalize function works correctly."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = GripperConfig(min_value=2.0, max_value=8.0)
                gripper = Gripper(gripper_config=config, spin_node=False)

                # Test unnormalization
                assert gripper._unnormalize(0.0) == 2.0  # min_value
                assert gripper._unnormalize(1.0) == 8.0  # max_value
                assert gripper._unnormalize(0.5) == 5.0  # middle value

    def test_gripper_normalize_unnormalize_roundtrip(self):
        """Test that normalize and unnormalize are inverse operations."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = GripperConfig(min_value=1.0, max_value=3.0)
                gripper = Gripper(gripper_config=config, spin_node=False)

                # Test roundtrip for various values
                test_values = [1.0, 1.5, 2.0, 2.5, 3.0]
                for value in test_values:
                    normalized = gripper._normalize(value)
                    unnormalized = gripper._unnormalize(normalized)
                    assert abs(unnormalized - value) < 1e-10


class TestGripperServices:
    """Test gripper service functionality."""

    def test_gripper_reboot_service_ready(self):
        """Test reboot method when service is ready."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(spin_node=False)

                # Mock service as ready
                gripper.reboot_client.service_is_ready = True

                # Test non-blocking call
                gripper.reboot(block=False)
                gripper.reboot_client.call_async.assert_called_once()

                # Test blocking call
                gripper.reboot(block=True)
                gripper.reboot_client.call.assert_called_once()

    def test_gripper_reboot_service_not_ready(self):
        """Test reboot method when service is not ready."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(spin_node=False)

                # Mock service as not ready
                gripper.reboot_client.service_is_ready = False

                with pytest.raises(RuntimeError, match="service.*is not available"):
                    gripper.reboot()

    def test_gripper_enable_torque_service_ready(self):
        """Test enable_torque method when service is ready."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(spin_node=False)

                # Mock service as ready
                gripper.enable_torque_client.service_is_ready = True

                # Test enable torque
                gripper.enable_torque(block=False)
                gripper.enable_torque_client.call_async.assert_called_once()

                # Test disable torque
                gripper.disable_torque(block=True)
                gripper.enable_torque_client.call.assert_called_once()

    def test_gripper_enable_torque_service_not_ready(self):
        """Test enable_torque method when service is not ready."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(spin_node=False)

                # Mock service as not ready
                gripper.enable_torque_client.service_is_ready = False

                with pytest.raises(RuntimeError, match="service.*is not available"):
                    gripper.enable_torque()

    def test_gripper_disable_torque_service_not_ready(self):
        """Test disable_torque method when service is not ready."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(spin_node=False)

                # Mock service as not ready
                gripper.enable_torque_client.service_is_ready = False

                with pytest.raises(RuntimeError, match="service.*is not available"):
                    gripper.disable_torque()


class TestGripperCallbacks:
    """Test gripper callback functionality."""

    def test_gripper_joint_state_callback(self):
        """Test joint state callback updates gripper state."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = GripperConfig(min_value=0.0, max_value=1.0, index=1)
                gripper = Gripper(gripper_config=config, spin_node=False)

                # Mock freshness checker
                gripper._joint_freshness_checker = Mock()

                # Create mock joint state message
                mock_msg = Mock()
                mock_msg.position = [0.1, 0.5, 0.8]  # gripper at index 1
                mock_msg.effort = [0.01, 0.05, 0.08]

                # Call the callback
                gripper._callback_joint_state(mock_msg)

                assert gripper._value == 0.5
                assert gripper._torque == 0.05
                gripper._joint_freshness_checker.update_timestamp.assert_called_once()

    def test_gripper_publish_target_callback_no_target(self):
        """Test publish target callback when no target is set."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_publisher = Mock()
                mock_node.create_publisher.return_value = mock_publisher
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                gripper = Gripper(spin_node=False)

                # Call callback with no target
                gripper._callback_publish_target()

                # Publisher should not be called
                mock_publisher.publish.assert_not_called()

    def test_gripper_publish_target_callback_with_target(self):
        """Test publish target callback with target set."""
        with patch("crisp_py.gripper.gripper.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.gripper.gripper.FreshnessChecker"):
                mock_node = Mock()
                mock_publisher = Mock()
                mock_node.create_publisher.return_value = mock_publisher
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_node.create_client.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = GripperConfig(min_value=0.0, max_value=1.0, max_delta=0.1)
                gripper = Gripper(gripper_config=config, spin_node=False)

                # Mock freshness checker
                gripper._joint_freshness_checker = Mock()

                # Set current value and target
                gripper._value = 0.5
                gripper._target = 0.8

                # Call callback
                gripper._callback_publish_target()

                # Publisher should be called
                mock_publisher.publish.assert_called_once()

                # Check the published message
                call_args = mock_publisher.publish.call_args[0][0]
                assert len(call_args.data) == 1
                # Value should be clipped by max_delta
                expected_value = 0.5 + 0.1  # current + max_delta
                assert abs(call_args.data[0] - expected_value) < 1e-10
