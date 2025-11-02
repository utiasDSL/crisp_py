"""Simplified unit tests for the Robot class focusing on core functionality."""

from unittest.mock import Mock, patch

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from crisp_py.robot import Pose, Robot, RobotConfig


class TestRobotBasics:
    """Test basic Robot class functionality without complex ROS dependencies."""

    def test_robot_threads_required(self):
        """Test that Robot class has correct thread requirements."""
        assert Robot.THREADS_REQUIRED == 4

    def test_robot_config_assignment(self):
        """Test that custom config is properly assigned."""
        with patch("crisp_py.robot.robot.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True
            mock_rclpy.create_node.return_value = Mock()

            # Mock all the ROS2 components
            with (
                patch("crisp_py.robot.robot.ControllerSwitcherClient"),
                patch("crisp_py.robot.robot.JointTrajectoryControllerClient"),
                patch("crisp_py.robot.robot.ParametersClient"),
                patch("crisp_py.robot.robot.CallbackMonitor"),
            ):
                # Mock node methods
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                custom_config = RobotConfig(joint_names=["j1", "j2"], home_config=[0.0, 0.1])

                robot = Robot(robot_config=custom_config, spin_node=False)

                assert robot.config == custom_config
                assert robot.nq == 2

    def test_robot_nq_property(self):
        """Test nq property returns correct number of joints."""
        with patch("crisp_py.robot.robot.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with (
                patch("crisp_py.robot.robot.ControllerSwitcherClient"),
                patch("crisp_py.robot.robot.JointTrajectoryControllerClient"),
                patch("crisp_py.robot.robot.ParametersClient"),
                patch("crisp_py.robot.robot.CallbackMonitor"),
            ):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = RobotConfig(joint_names=["j1", "j2", "j3"], home_config=[0, 0, 0])
                robot = Robot(robot_config=config, spin_node=False)

                assert robot.nq == 3


class TestRobotReadinessChecks:
    """Test robot readiness and state management."""

    def test_robot_is_ready_false_initially(self):
        """Test robot is not ready initially."""
        with patch("crisp_py.robot.robot.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with (
                patch("crisp_py.robot.robot.ControllerSwitcherClient"),
                patch("crisp_py.robot.robot.JointTrajectoryControllerClient"),
                patch("crisp_py.robot.robot.ParametersClient"),
                patch("crisp_py.robot.robot.CallbackMonitor"),
            ):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                robot = Robot(spin_node=False)

                assert not robot.is_ready()

    def test_robot_is_ready_true_when_all_set(self):
        """Test robot is ready when all required values are set."""
        with patch("crisp_py.robot.robot.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with (
                patch("crisp_py.robot.robot.ControllerSwitcherClient"),
                patch("crisp_py.robot.robot.JointTrajectoryControllerClient"),
                patch("crisp_py.robot.robot.ParametersClient"),
                patch("crisp_py.robot.robot.CallbackMonitor"),
            ):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                robot = Robot(spin_node=False)

                # Manually set the required values
                robot._current_pose = Pose(np.array([0, 0, 0]), Rotation.identity())
                robot._target_pose = Pose(np.array([0, 0, 0]), Rotation.identity())
                robot._current_joint = np.array([0, 0, 0])
                robot._target_joint = np.array([0, 0, 0])

                assert robot.is_ready()

    def test_robot_reset_targets(self):
        """Test reset_targets method."""
        with patch("crisp_py.robot.robot.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with (
                patch("crisp_py.robot.robot.ControllerSwitcherClient"),
                patch("crisp_py.robot.robot.JointTrajectoryControllerClient"),
                patch("crisp_py.robot.robot.ParametersClient"),
                patch("crisp_py.robot.robot.CallbackMonitor"),
            ):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                robot = Robot(spin_node=False)

                # Set some targets
                robot._target_pose = Pose(np.array([1, 2, 3]), Rotation.identity())
                robot._target_joint = np.array([0.1, 0.2, 0.3])
                robot._target_wrench = {"force": np.array([1, 2, 3]), "torque": np.array([1, 2, 3])}

                robot.reset_targets()

                assert robot._target_pose is None
                assert robot._target_joint is None
                assert robot._target_wrench is None


class TestRobotPropertyErrors:
    """Test robot property access error handling."""

    def test_end_effector_pose_error_when_none(self):
        """Test end_effector_pose raises error when not set."""
        with patch("crisp_py.robot.robot.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with (
                patch("crisp_py.robot.robot.ControllerSwitcherClient"),
                patch("crisp_py.robot.robot.JointTrajectoryControllerClient"),
                patch("crisp_py.robot.robot.ParametersClient"),
                patch("crisp_py.robot.robot.CallbackMonitor"),
            ):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                robot = Robot(spin_node=False)

                with pytest.raises(RuntimeError, match="The robot has not received any poses yet"):
                    robot.end_effector_pose

    def test_joint_values_error_when_none(self):
        """Test joint_values raises error when not set."""
        with patch("crisp_py.robot.robot.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with (
                patch("crisp_py.robot.robot.ControllerSwitcherClient"),
                patch("crisp_py.robot.robot.JointTrajectoryControllerClient"),
                patch("crisp_py.robot.robot.ParametersClient"),
                patch("crisp_py.robot.robot.CallbackMonitor"),
            ):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                robot = Robot(spin_node=False)

                with pytest.raises(RuntimeError, match="The robot has not received any joints yet"):
                    robot.joint_values


class TestRobotTargetSetting:
    """Test robot target setting functionality."""

    def test_set_target_joint_correct_size(self):
        """Test set_target_joint with correct size."""
        with patch("crisp_py.robot.robot.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with (
                patch("crisp_py.robot.robot.ControllerSwitcherClient"),
                patch("crisp_py.robot.robot.JointTrajectoryControllerClient"),
                patch("crisp_py.robot.robot.ParametersClient"),
                patch("crisp_py.robot.robot.CallbackMonitor"),
            ):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = RobotConfig(joint_names=["j1", "j2"], home_config=[0, 0])
                robot = Robot(robot_config=config, spin_node=False)

                joint_values = np.array([0.1, 0.2])
                robot.set_target_joint(joint_values)

                assert np.array_equal(robot._target_joint, joint_values)

    def test_set_target_joint_wrong_size(self):
        """Test set_target_joint with wrong size."""
        with patch("crisp_py.robot.robot.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with (
                patch("crisp_py.robot.robot.ControllerSwitcherClient"),
                patch("crisp_py.robot.robot.JointTrajectoryControllerClient"),
                patch("crisp_py.robot.robot.ParametersClient"),
                patch("crisp_py.robot.robot.CallbackMonitor"),
            ):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = RobotConfig(joint_names=["j1", "j2"], home_config=[0, 0])
                robot = Robot(robot_config=config, spin_node=False)

                joint_values = np.array([0.1, 0.2, 0.3])  # Wrong size

                with pytest.raises(AssertionError, match="Joint state must be of size nq"):
                    robot.set_target_joint(joint_values)

    def test_set_target_wrench_defaults(self):
        """Test set_target_wrench with default values."""
        with patch("crisp_py.robot.robot.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with (
                patch("crisp_py.robot.robot.ControllerSwitcherClient"),
                patch("crisp_py.robot.robot.JointTrajectoryControllerClient"),
                patch("crisp_py.robot.robot.ParametersClient"),
                patch("crisp_py.robot.robot.CallbackMonitor"),
            ):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                robot = Robot(spin_node=False)
                robot.set_target_wrench()

                expected_force = np.array([0.0, 0.0, 0.0])
                expected_torque = np.array([0.0, 0.0, 0.0])

                assert np.array_equal(robot._target_wrench["force"], expected_force)
                assert np.array_equal(robot._target_wrench["torque"], expected_torque)

    def test_set_target_wrench_wrong_size(self):
        """Test set_target_wrench with wrong size vectors."""
        with patch("crisp_py.robot.robot.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with (
                patch("crisp_py.robot.robot.ControllerSwitcherClient"),
                patch("crisp_py.robot.robot.JointTrajectoryControllerClient"),
                patch("crisp_py.robot.robot.ParametersClient"),
                patch("crisp_py.robot.robot.CallbackMonitor"),
            ):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                robot = Robot(spin_node=False)

                # Test wrong force size
                with pytest.raises(AssertionError, match="Force must be a 3D vector"):
                    robot.set_target_wrench(force=[1.0, 2.0])

                # Test wrong torque size
                with pytest.raises(AssertionError, match="Torque must be a 3D vector"):
                    robot.set_target_wrench(torque=[1.0, 2.0, 3.0, 4.0])


class TestRobotUtilities:
    """Test robot utility functions."""

    def test_parse_pose_or_position_neither(self):
        """Test _parse_pose_or_position with neither position nor pose."""
        with patch("crisp_py.robot.robot.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with (
                patch("crisp_py.robot.robot.ControllerSwitcherClient"),
                patch("crisp_py.robot.robot.JointTrajectoryControllerClient"),
                patch("crisp_py.robot.robot.ParametersClient"),
                patch("crisp_py.robot.robot.CallbackMonitor"),
            ):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                robot = Robot(spin_node=False)

                with pytest.raises(
                    AssertionError, match="Either position or pose must be provided"
                ):
                    robot._parse_pose_or_position()

    def test_parse_pose_or_position_wrong_position_size(self):
        """Test _parse_pose_or_position with wrong position size."""
        with patch("crisp_py.robot.robot.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with (
                patch("crisp_py.robot.robot.ControllerSwitcherClient"),
                patch("crisp_py.robot.robot.JointTrajectoryControllerClient"),
                patch("crisp_py.robot.robot.ParametersClient"),
                patch("crisp_py.robot.robot.CallbackMonitor"),
            ):
                mock_node = Mock()
                mock_node.create_publisher.return_value = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_node.create_timer.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                robot = Robot(spin_node=False)

                # Set up a valid target pose first
                robot._target_pose = Pose(np.array([0, 0, 0]), Rotation.identity())

                position = [1.0, 2.0]  # Wrong size

                with pytest.raises(AssertionError, match="Position must be a 3D vector"):
                    robot._parse_pose_or_position(position=position)
