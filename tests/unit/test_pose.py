"""Unit tests for the Pose class."""

import numpy as np
from scipy.spatial.transform import Rotation

from crisp_py.robot import Pose


class TestPose:
    """Test cases for the Pose class."""

    def test_pose_creation(self):
        """Test basic pose creation."""
        position = np.array([1.0, 2.0, 3.0])
        orientation = Rotation.identity()
        pose = Pose(position, orientation)

        assert np.array_equal(pose.position, position)
        assert pose.orientation.as_quat().tolist() == orientation.as_quat().tolist()

    def test_pose_copy(self):
        """Test pose copy functionality."""
        position = np.array([1.0, 2.0, 3.0])
        orientation = Rotation.from_euler("xyz", [0.1, 0.2, 0.3])
        pose = Pose(position, orientation)

        copied_pose = pose.copy()

        # Check that values are equal
        assert np.array_equal(pose.position, copied_pose.position)
        assert np.allclose(pose.orientation.as_quat(), copied_pose.orientation.as_quat())

        # Check that they are different objects
        assert pose is not copied_pose
        assert pose.position is not copied_pose.position

    def test_pose_copy_modification(self):
        """Test that modifying a copy doesn't affect the original."""
        position = np.array([1.0, 2.0, 3.0])
        orientation = Rotation.identity()
        pose = Pose(position, orientation)

        copied_pose = pose.copy()
        copied_pose.position[0] = 5.0

        # Original should be unchanged
        assert pose.position[0] == 1.0
        assert copied_pose.position[0] == 5.0

    def test_pose_str_representation(self):
        """Test string representation of pose."""
        position = np.array([1.0, 2.0, 3.0])
        orientation = Rotation.identity()
        pose = Pose(position, orientation)

        str_repr = str(pose)

        assert "Pos:" in str_repr
        assert "Orientation:" in str_repr
        assert "1.00" in str_repr
        assert "2.00" in str_repr
        assert "3.00" in str_repr

    def test_pose_subtraction(self):
        """Test pose subtraction (relative pose computation)."""
        pos1 = np.array([2.0, 3.0, 4.0])
        pos2 = np.array([1.0, 1.0, 1.0])
        rot1 = Rotation.from_euler("z", 0.5)
        rot2 = Rotation.from_euler("z", 0.2)

        pose1 = Pose(pos1, rot1)
        pose2 = Pose(pos2, rot2)

        relative_pose = pose1 - pose2

        expected_position = pos1 - pos2
        expected_rotation = rot1 * rot2.inv()

        assert np.allclose(relative_pose.position, expected_position)
        assert np.allclose(relative_pose.orientation.as_quat(), expected_rotation.as_quat())

    def test_pose_addition(self):
        """Test pose addition (adding relative pose)."""
        pos1 = np.array([1.0, 2.0, 3.0])
        pos2 = np.array([0.5, 0.5, 0.5])
        rot1 = Rotation.from_euler("z", 0.3)
        rot2 = Rotation.from_euler("z", 0.2)

        pose1 = Pose(pos1, rot1)
        pose2 = Pose(pos2, rot2)

        result_pose = pose1 + pose2

        expected_position = pos1 + pos2
        expected_rotation = rot2 * rot1

        assert np.allclose(result_pose.position, expected_position)
        assert np.allclose(result_pose.orientation.as_quat(), expected_rotation.as_quat())

    def test_pose_operations_with_identity(self):
        """Test pose operations with identity transformations."""
        position = np.array([1.0, 2.0, 3.0])
        orientation = Rotation.from_euler("xyz", [0.1, 0.2, 0.3])
        pose = Pose(position, orientation)

        identity_pose = Pose(np.zeros(3), Rotation.identity())

        # Adding identity should preserve rotation and add position
        result_add = pose + identity_pose
        assert np.allclose(result_add.position, position)
        assert np.allclose(result_add.orientation.as_quat(), orientation.as_quat())

        # Subtracting identity should preserve relative position and orientation
        result_sub = pose - identity_pose
        assert np.allclose(result_sub.position, position)
        assert np.allclose(result_sub.orientation.as_quat(), orientation.as_quat())

    def test_pose_with_different_rotations(self):
        """Test pose operations with different rotation representations."""
        position = np.array([1.0, 2.0, 3.0])

        # Test with quaternion
        quat = [0.0, 0.0, 0.707, 0.707]  # 90 degrees around z
        rot_quat = Rotation.from_quat(quat)
        pose_quat = Pose(position, rot_quat)

        # Test with euler angles
        rot_euler = Rotation.from_euler("z", np.pi / 2)
        pose_euler = Pose(position, rot_euler)

        # Should be approximately equal
        assert np.allclose(
            pose_quat.orientation.as_quat(), pose_euler.orientation.as_quat(), atol=1e-6
        )

    def test_pose_position_types(self):
        """Test pose creation with different position types."""
        # Test with list
        position_list = [1.0, 2.0, 3.0]
        orientation = Rotation.identity()
        pose_list = Pose(np.array(position_list), orientation)

        # Test with numpy array
        position_array = np.array([1.0, 2.0, 3.0])
        pose_array = Pose(position_array, orientation)

        assert np.array_equal(pose_list.position, pose_array.position)
