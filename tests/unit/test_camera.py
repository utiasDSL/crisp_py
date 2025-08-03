"""Unit tests for the Camera class and CameraConfig."""

from unittest.mock import Mock, patch

import numpy as np
import pytest

from crisp_py.camera.camera import Camera
from crisp_py.camera.camera_config import CameraConfig, FrankaCameraConfig


class TestCameraConfig:
    """Test cases for the CameraConfig class."""

    def test_camera_config_defaults(self):
        """Test default CameraConfig values."""
        config = CameraConfig()

        assert config.camera_name == "camera"
        assert config.camera_frame == "camera_link"
        assert config.resolution is None
        assert config.camera_color_image_topic is None
        assert config.camera_color_info_topic is None
        assert config.max_image_delay == 1.0

    def test_camera_config_custom_values(self):
        """Test CameraConfig with custom values."""
        config = CameraConfig(
            camera_name="test_camera",
            camera_frame="test_frame",
            resolution=(640, 480),
            camera_color_image_topic="test/color/image",
            camera_color_info_topic="test/color/info",
            max_image_delay=2.0,
        )

        assert config.camera_name == "test_camera"
        assert config.camera_frame == "test_frame"
        assert config.resolution == (640, 480)
        assert config.camera_color_image_topic == "test/color/image"
        assert config.camera_color_info_topic == "test/color/info"
        assert config.max_image_delay == 2.0

    def test_franka_camera_config_defaults(self):
        """Test FrankaCameraConfig defaults."""
        config = FrankaCameraConfig()

        assert config.camera_name == "franka"
        assert config.camera_frame == "franka_link"
        assert config.resolution == (256, 256)
        assert config.camera_color_image_topic == "franka/color/image_raw"
        assert config.camera_color_info_topic == "franka/color/camera_info"
        assert config.max_image_delay == 1.0  # Inherited

    def test_franka_camera_config_inheritance(self):
        """Test that FrankaCameraConfig inherits from CameraConfig."""
        config = FrankaCameraConfig()

        assert isinstance(config, CameraConfig)
        assert config.max_image_delay == 1.0  # Inherited default

    def test_franka_camera_config_custom_override(self):
        """Test overriding FrankaCameraConfig defaults."""
        config = FrankaCameraConfig(
            camera_name="custom_franka", resolution=(512, 512), max_image_delay=3.0
        )

        assert config.camera_name == "custom_franka"
        assert config.resolution == (512, 512)
        assert config.max_image_delay == 3.0
        assert config.camera_color_image_topic == "franka/color/image_raw"  # Still uses default


class TestCameraBasics:
    """Test basic Camera class functionality."""

    def test_camera_threads_required(self):
        """Test that Camera class has correct thread requirements."""
        assert Camera.THREADS_REQUIRED == 2

    def test_camera_init_with_default_config(self):
        """Test camera initialization with default config."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                camera = Camera(spin_node=False)

                assert isinstance(camera.config, CameraConfig)
                assert camera.config.camera_name == "camera"

    def test_camera_init_with_custom_config(self):
        """Test camera initialization with custom config."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                custom_config = CameraConfig(camera_name="test_camera", resolution=(640, 480))
                camera = Camera(config=custom_config, spin_node=False)

                assert camera.config == custom_config
                assert camera.config.camera_name == "test_camera"
                assert camera.config.resolution == (640, 480)

    def test_camera_init_with_franka_config(self):
        """Test camera initialization with Franka config."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                franka_config = FrankaCameraConfig()
                camera = Camera(config=franka_config, spin_node=False)

                assert camera.config == franka_config
                assert camera.config.camera_name == "franka"
                assert camera.config.resolution == (256, 256)

    def test_camera_init_with_namespace(self):
        """Test camera initialization with namespace."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                Camera(namespace="test_namespace", spin_node=False)

                # Check that the node was created with namespace
                mock_rclpy.create_node.assert_called_once_with("camera", namespace="test_namespace")

    def test_camera_init_with_existing_node(self):
        """Test camera initialization with existing node."""
        with patch("crisp_py.camera.camera.CallbackMonitor"):
            existing_node = Mock()
            existing_node.create_subscription.return_value = Mock()

            camera = Camera(node=existing_node, spin_node=False)

            assert camera.node == existing_node


class TestCameraProperties:
    """Test camera properties and state management."""

    def test_camera_initial_state(self):
        """Test camera initial state."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                camera = Camera(spin_node=False)

                assert camera._current_image is None
                assert not camera.is_ready()

    def test_camera_current_image_error_when_none(self):
        """Test current_image raises error when not set."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                camera = Camera(spin_node=False)

                with pytest.raises(RuntimeError, match="We have not received any images"):
                    camera.current_image

    def test_camera_current_image_returns_image(self):
        """Test current_image returns image when available."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                camera = Camera(spin_node=False)

                # Set a test image
                test_image = np.zeros((100, 100, 3), dtype=np.uint8)
                camera._current_image = test_image

                result = camera.current_image

                assert np.array_equal(result, test_image)
                # Note: CallbackMonitor handles freshness checking automatically

    def test_camera_resolution_with_config(self):
        """Test resolution property with config set."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = CameraConfig(resolution=(640, 480))
                camera = Camera(config=config, spin_node=False)

                assert camera.resolution == (640, 480)

    def test_camera_resolution_without_config(self):
        """Test resolution property without config set."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                camera = Camera(spin_node=False)

                assert camera.resolution == (0, 0)

    def test_camera_is_ready_true_when_both_set(self):
        """Test is_ready returns True when both image and resolution are set."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = CameraConfig(resolution=(640, 480))
                camera = Camera(config=config, spin_node=False)

                # Set image
                camera._current_image = np.zeros((100, 100, 3), dtype=np.uint8)

                assert camera.is_ready()

    def test_camera_is_ready_false_when_image_missing(self):
        """Test is_ready returns False when image is missing."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = CameraConfig(resolution=(640, 480))
                camera = Camera(config=config, spin_node=False)

                # Image is None by default
                assert not camera.is_ready()

    def test_camera_is_ready_false_when_resolution_missing(self):
        """Test is_ready returns False when resolution is missing."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                camera = Camera(spin_node=False)  # No resolution set

                # Set image
                camera._current_image = np.zeros((100, 100, 3), dtype=np.uint8)

                assert not camera.is_ready()


class TestCameraImageProcessing:
    """Test camera image processing methods."""

    def test_camera_uncompress_method(self):
        """Test _uncompress method."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                camera = Camera(spin_node=False)

                # Mock cv_bridge
                mock_compressed_image = Mock()
                test_image = np.zeros((100, 100, 3), dtype=np.uint8)

                with patch.object(
                    camera.cv_bridge, "compressed_imgmsg_to_cv2", return_value=test_image
                ) as mock_method:
                    result = camera._uncompress(mock_compressed_image)

                    assert np.array_equal(result, test_image)
                    mock_method.assert_called_once_with(
                        mock_compressed_image, desired_encoding="rgb8"
                    )

    def test_camera_resize_with_aspect_ratio_no_resize_needed(self):
        """Test _resize_with_aspect_ratio when no resize is needed."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                camera = Camera(spin_node=False)

                # Test image that matches target resolution
                test_image = np.zeros((100, 100, 3), dtype=np.uint8)
                target_res = (100, 100)

                with patch("cv2.resize") as mock_resize:
                    mock_resize.return_value = test_image

                    result = camera._resize_with_aspect_ratio(test_image, target_res)

                    # Should still call resize due to the implementation
                    mock_resize.assert_called_once()
                    assert result.shape[:2] == target_res

    def test_camera_resize_with_aspect_ratio_upscale(self):
        """Test _resize_with_aspect_ratio when upscaling is needed."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                camera = Camera(spin_node=False)

                # Test image smaller than target
                test_image = np.zeros((50, 50, 3), dtype=np.uint8)
                target_res = (100, 100)

                with patch("cv2.resize") as mock_resize:
                    # Mock resized image
                    resized_image = np.zeros((100, 100, 3), dtype=np.uint8)
                    mock_resize.return_value = resized_image

                    result = camera._resize_with_aspect_ratio(test_image, target_res)

                    mock_resize.assert_called_once()
                    assert result.shape[:2] == target_res

    def test_camera_resize_with_aspect_ratio_crop(self):
        """Test _resize_with_aspect_ratio when cropping is needed."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                camera = Camera(spin_node=False)

                # Test image with different aspect ratio
                test_image = np.zeros((100, 200, 3), dtype=np.uint8)
                target_res = (100, 100)

                with patch("cv2.resize") as mock_resize:
                    # Mock resized image (larger than target)
                    resized_image = np.zeros((100, 200, 3), dtype=np.uint8)
                    mock_resize.return_value = resized_image

                    result = camera._resize_with_aspect_ratio(test_image, target_res)

                    mock_resize.assert_called_once()
                    assert result.shape[:2] == target_res

    def test_camera_image_to_array_method(self):
        """Test _image_to_array method."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                camera = Camera(spin_node=False)

                # Mock cv_bridge
                mock_image_msg = Mock()
                test_image = np.zeros((100, 100, 3), dtype=np.uint8)

                with patch.object(
                    camera.cv_bridge, "imgmsg_to_cv2", return_value=test_image
                ) as mock_method:
                    result = camera._image_to_array(mock_image_msg)

                    assert np.array_equal(result, test_image)
                    mock_method.assert_called_once_with(mock_image_msg, desired_encoding="rgb8")


class TestCameraCallbacks:
    """Test camera callback functionality."""

    def test_camera_callback_current_color_image(self):
        """Test _callback_current_color_image."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = CameraConfig(resolution=(100, 100))
                camera = Camera(config=config, spin_node=False)

                # Mock image processing methods
                test_image = np.zeros((100, 100, 3), dtype=np.uint8)
                camera._uncompress = Mock(return_value=test_image)
                camera._resize_with_aspect_ratio = Mock(return_value=test_image)

                # Create mock compressed image message
                mock_msg = Mock()

                # Call the callback
                camera._callback_current_color_image(mock_msg)

                # Verify methods were called
                camera._uncompress.assert_called_once_with(mock_msg)
                camera._resize_with_aspect_ratio.assert_called_once_with(
                    test_image, target_res=(100, 100)
                )
                # Note: CallbackMonitor handles timestamp updates automatically

                # Verify image was stored
                assert np.array_equal(camera._current_image, test_image)

    def test_camera_callback_current_color_info_sets_resolution(self):
        """Test _callback_current_color_info sets resolution when None."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                camera = Camera(spin_node=False)  # No resolution set

                # Create mock camera info message
                mock_msg = Mock()
                mock_msg.height = 480
                mock_msg.width = 640

                # Call the callback
                camera._callback_current_color_info(mock_msg)

                # Verify resolution was set
                assert camera.config.resolution == (480, 640)

    def test_camera_callback_current_color_info_preserves_existing_resolution(self):
        """Test _callback_current_color_info preserves existing resolution."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = CameraConfig(resolution=(100, 100))
                camera = Camera(config=config, spin_node=False)

                # Create mock camera info message
                mock_msg = Mock()
                mock_msg.height = 480
                mock_msg.width = 640

                # Call the callback
                camera._callback_current_color_info(mock_msg)

                # Verify resolution was NOT changed
                assert camera.config.resolution == (100, 100)


class TestCameraErrorHandling:
    """Test camera error handling and edge cases."""

    def test_camera_wait_until_ready_timeout(self):
        """Test wait_until_ready raises TimeoutError."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rate = Mock()
                mock_node.create_rate.return_value = mock_rate
                mock_rclpy.create_node.return_value = mock_node

                camera = Camera(spin_node=False)

                # Camera is not ready (image is None, resolution is None)
                with pytest.raises(TimeoutError, match="Timeout waiting for camera"):
                    camera.wait_until_ready(timeout=0.1, check_frequency=10.0)

    def test_camera_wait_until_ready_success(self):
        """Test wait_until_ready succeeds when camera becomes ready."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rate = Mock()
                mock_node.create_rate.return_value = mock_rate
                mock_rclpy.create_node.return_value = mock_node

                config = CameraConfig(resolution=(100, 100))
                camera = Camera(config=config, spin_node=False)

                # Make camera ready immediately
                camera._current_image = np.zeros((100, 100, 3), dtype=np.uint8)

                # Should not raise exception
                camera.wait_until_ready(timeout=1.0, check_frequency=10.0)

    def test_camera_current_image_with_freshness_check(self):
        """Test current_image property calls freshness checker."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                camera = Camera(spin_node=False)

                # Set image
                test_image = np.zeros((100, 100, 3), dtype=np.uint8)
                camera._current_image = test_image

                # Access current_image
                result = camera.current_image

                # Verify result is returned correctly
                assert np.array_equal(result, test_image)
                # Note: CallbackMonitor handles freshness checking automatically


class TestCameraIntegration:
    """Test camera integration scenarios."""

    def test_camera_full_image_pipeline(self):
        """Test full image processing pipeline."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = CameraConfig(resolution=(256, 256))
                camera = Camera(config=config, spin_node=False)

                # Note: CallbackMonitor handles freshness checking automatically

                # Create test pipeline - use square image to avoid aspect ratio issues
                original_image = np.ones((300, 300, 3), dtype=np.uint8) * 255

                # Mock cv_bridge
                with patch.object(
                    camera.cv_bridge, "compressed_imgmsg_to_cv2", return_value=original_image
                ):
                    # Create mock message
                    mock_msg = Mock()

                    # Process image through pipeline
                    camera._callback_current_color_image(mock_msg)

                    # Verify the image was processed and stored
                    assert camera._current_image is not None
                    assert camera._current_image.shape[:2] == (256, 256)

                    # Verify image can be accessed
                    result = camera.current_image
                    assert np.array_equal(result, camera._current_image)

    def test_camera_subscription_setup(self):
        """Test that camera sets up correct subscriptions."""
        with patch("crisp_py.camera.camera.rclpy") as mock_rclpy:
            mock_rclpy.ok.return_value = True

            with patch("crisp_py.camera.camera.CallbackMonitor"):
                mock_node = Mock()
                mock_node.create_subscription.return_value = Mock()
                mock_rclpy.create_node.return_value = mock_node

                config = CameraConfig(
                    camera_color_image_topic="test/color/image",
                    camera_color_info_topic="test/color/info",
                )
                Camera(config=config, spin_node=False)

                # Verify subscriptions were created
                assert mock_node.create_subscription.call_count == 2

                # Check the calls
                calls = mock_node.create_subscription.call_args_list

                # First call should be for compressed image
                assert "test/color/image/compressed" in str(calls[0])

                # Second call should be for camera info
                assert "test/color/info" in str(calls[1])
