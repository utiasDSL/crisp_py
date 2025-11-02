"""Unit tests for robot configuration classes."""

import numpy as np

from crisp_py.robot.robot_config import (
    FrankaConfig,
    IiwaConfig,
    KinovaConfig,
    RobotConfig,
    SO101Config,
)


class TestRobotConfig:
    """Test cases for the base RobotConfig class."""

    def test_robot_config_creation(self):
        """Test basic robot config creation."""
        joint_names = ["joint1", "joint2", "joint3"]
        home_config = [0.0, 0.1, 0.2]

        config = RobotConfig(joint_names=joint_names, home_config=home_config)

        assert config.joint_names == joint_names
        assert config.home_config == home_config
        assert config.base_frame == "base_link"
        assert config.target_frame == "end_effector_link"
        assert config.publish_frequency == 50.0

    def test_robot_config_defaults(self):
        """Test default values in RobotConfig."""
        config = RobotConfig(joint_names=["j1"], home_config=[0.0])

        assert config.default_controller == "cartesian_impedance_controller"
        assert config.cartesian_impedance_controller_name == "cartesian_impedance_controller"
        assert config.joint_trajectory_controller_name == "joint_impedance_controller"
        assert config.target_pose_topic == "target_pose"
        assert config.target_joint_topic == "target_joint"
        assert config.current_pose_topic == "current_pose"
        assert config.current_joint_topic == "joint_states"
        assert config.time_to_home == 5.0
        assert config.max_pose_delay == 1.0
        assert config.max_joint_delay == 1.0

    def test_robot_config_custom_values(self):
        """Test RobotConfig with custom values."""
        config = RobotConfig(
            joint_names=["custom_joint"],
            home_config=[0.5],
            base_frame="custom_base",
            target_frame="custom_ee",
            publish_frequency=100.0,
            time_to_home=10.0,
        )

        assert config.base_frame == "custom_base"
        assert config.target_frame == "custom_ee"
        assert config.publish_frequency == 100.0
        assert config.time_to_home == 10.0


class TestFrankaConfig:
    """Test cases for the FrankaConfig class."""

    def test_franka_config_defaults(self):
        """Test default values in FrankaConfig."""
        config = FrankaConfig()

        assert len(config.joint_names) == 7
        assert len(config.home_config) == 7
        assert config.base_frame == "base"
        assert config.target_frame == "fr3_hand_tcp"

        # Check joint names
        expected_joints = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]
        assert config.joint_names == expected_joints

    def test_franka_config_home_positions(self):
        """Test Franka home configuration values."""
        config = FrankaConfig()

        expected_home = [
            0,
            -np.pi / 4,
            0,
            -3 * np.pi / 4,
            0,
            np.pi / 2,
            np.pi / 4,
        ]

        assert np.allclose(config.home_config, expected_home)

    def test_franka_config_inheritance(self):
        """Test that FrankaConfig inherits from RobotConfig."""
        config = FrankaConfig()

        assert isinstance(config, RobotConfig)
        assert config.publish_frequency == 50.0  # Inherited default
        assert config.time_to_home == 5.0  # Inherited default

    def test_franka_config_custom_override(self):
        """Test overriding FrankaConfig defaults."""
        custom_joints = ["custom_joint1", "custom_joint2"]
        custom_home = [0.1, 0.2]

        config = FrankaConfig(
            joint_names=custom_joints, home_config=custom_home, base_frame="custom_base"
        )

        assert config.joint_names == custom_joints
        assert config.home_config == custom_home
        assert config.base_frame == "custom_base"
        assert config.target_frame == "fr3_hand_tcp"  # Still uses Franka default


class TestKinovaConfig:
    """Test cases for the KinovaConfig class."""

    def test_kinova_config_defaults(self):
        """Test default values in KinovaConfig."""
        config = KinovaConfig()

        assert len(config.joint_names) == 7
        assert len(config.home_config) == 7

        expected_joints = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7",
        ]
        assert config.joint_names == expected_joints

    def test_kinova_config_home_positions(self):
        """Test Kinova home configuration values."""
        config = KinovaConfig()

        expected_home = [
            0,
            -np.pi / 4,
            0,
            -3 * np.pi / 4,
            0,
            np.pi / 2,
            np.pi / 4,
        ]

        assert np.allclose(config.home_config, expected_home)

    def test_kinova_config_uses_base_defaults(self):
        """Test that KinovaConfig uses RobotConfig defaults for frames."""
        config = KinovaConfig()

        assert config.base_frame == "base_link"
        assert config.target_frame == "end_effector_link"


class TestIiwaConfig:
    """Test cases for the IiwaConfig class."""

    def test_iiwa_config_defaults(self):
        """Test default values in IiwaConfig."""
        config = IiwaConfig()

        assert len(config.joint_names) == 7
        assert len(config.home_config) == 7
        assert config.base_frame == "iiwa_base"
        assert config.target_frame == "tool0"

        expected_joints = [
            "joint_a1",
            "joint_a2",
            "joint_a3",
            "joint_a4",
            "joint_a5",
            "joint_a6",
            "joint_a7",
        ]
        assert config.joint_names == expected_joints

    def test_iiwa_config_home_positions(self):
        """Test Iiwa home configuration values."""
        config = IiwaConfig()

        expected_home = [
            0,
            -np.pi / 4,
            0,
            -3 * np.pi / 4,
            0,
            np.pi / 2,
            np.pi / 4,
        ]

        assert np.allclose(config.home_config, expected_home)


class TestSO101Config:
    """Test cases for the SO101Config class."""

    def test_so101_config_defaults(self):
        """Test default values in SO101Config."""
        config = SO101Config()

        assert len(config.joint_names) == 6
        assert len(config.home_config) == 6
        assert config.base_frame == "Base"
        assert config.target_frame == "Fixed_Gripper"

        expected_joints = [
            "Shoulder_Rotation",
            "Shoulder_Pitch",
            "Elbow",
            "Wrist_Pitch",
            "Wrist_Roll",
            "Gripper",
        ]
        assert config.joint_names == expected_joints

    def test_so101_config_home_positions(self):
        """Test SO101 home configuration values."""
        config = SO101Config()

        expected_home = [0, 0, 0, 0, 0, 0]
        assert config.home_config == expected_home


class TestConfigConsistency:
    """Test consistency across different robot configurations."""

    def test_all_configs_have_matching_joint_and_home_lengths(self):
        """Test that all configs have matching joint_names and home_config lengths."""
        configs = [FrankaConfig(), KinovaConfig(), IiwaConfig(), SO101Config()]

        for config in configs:
            assert len(config.joint_names) == len(config.home_config), (
                f"Mismatch in {config.__class__.__name__}: "
                f"joints={len(config.joint_names)}, home={len(config.home_config)}"
            )

    def test_all_configs_inherit_from_robot_config(self):
        """Test that all specific configs inherit from RobotConfig."""
        configs = [FrankaConfig(), KinovaConfig(), IiwaConfig(), SO101Config()]

        for config in configs:
            assert isinstance(config, RobotConfig)

    def test_all_configs_have_required_attributes(self):
        """Test that all configs have required attributes."""
        configs = [FrankaConfig(), KinovaConfig(), IiwaConfig(), SO101Config()]

        required_attrs = [
            "joint_names",
            "home_config",
            "base_frame",
            "target_frame",
            "default_controller",
            "cartesian_impedance_controller_name",
            "joint_trajectory_controller_name",
            "target_pose_topic",
            "target_joint_topic",
            "current_pose_topic",
            "current_joint_topic",
            "publish_frequency",
            "time_to_home",
            "max_pose_delay",
            "max_joint_delay",
        ]

        for config in configs:
            for attr in required_attrs:
                assert hasattr(config, attr), f"{config.__class__.__name__} missing {attr}"
