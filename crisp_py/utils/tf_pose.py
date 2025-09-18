"""Util class to get a pose from the tf2 instead of using a dedicated topic."""

from rclpy.node import Node
from rclpy.time import Time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from crisp_py.utils.geometry import Pose


class TfPose:
    """Class to get a pose from the tf2 instead of using a dedicated topic."""

    def __init__(
        self, node: Node, target_frame: str, source_frame: str, retrieve_rate: float = 50.0
    ):
        """Initialize the TfPose class.

        Args:
            node (Node): The ROS2 node.
            target_frame (str): The target frame to get the pose of.
            source_frame (str): The source frame to get the pose relative to.
            retrieve_rate (float, optional): The rate at which to retrieve the pose. Defaults to 50.0.
        """
        self.node = node
        self.target_frame = target_frame
        self.source_frame = source_frame
        self.retrieve_rate = retrieve_rate

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.timer = self.node.create_timer(1.0 / self.retrieve_rate, self._callback_retrieve_pose)

        self.current_pose: Pose | None = None

    @property
    def pose(self) -> Pose:
        """Get the current pose."""
        if self.current_pose is None:
            raise ValueError("Pose has not been retrieved yet.")
        return self.current_pose

    def _callback_retrieve_pose(self):
        """Retrieve the latest transform between source_frame and target_frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.source_frame, self.target_frame, Time()
            )
        except Exception as e:
            self.node.get_logger().warn(f"Could not get transform: {e}", throttle_duration_sec=5.0)
            return
        self.current_pose = Pose.from_transform_msg(transform)
