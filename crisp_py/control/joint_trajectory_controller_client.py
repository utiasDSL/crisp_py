"""TODO: Add a description here."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class JointTrajectoryControllerClient(ActionClient):
    def __init__(self, node: Node) -> None:
        self.node = node
        super().__init__(
            node,
            FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
        )
        self._goal = FollowJointTrajectory.Goal()
        self._prefix = (
            f"{self.node.get_namespace().strip('/')}_"
            if self.node.get_namespace().strip('/') != ""
            else ""
        )

    def send_joint_config(
        self,
        joint_names: list,
        joint_config: list,
        time_to_goal: float = 5.0,
        blocking: bool = True,
    ):
        """Send joint configuration to the robot."""
        self._goal.trajectory.joint_names = [
            self._prefix + joint_name for joint_name in joint_names
        ]
        self._goal.trajectory.header.stamp = self.node.get_clock().now().to_msg()
        self._goal.trajectory.points = []
        self._goal.trajectory.points.append(
            JointTrajectoryPoint(
                positions=joint_config,
                velocities=len(joint_config) * [0.0],
                accelerations=len(joint_config) * [0.0],
                time_from_start=rclpy.duration.Duration(
                    seconds=time_to_goal, nanoseconds=0
                ).to_msg(),
            )
        )
        future = self.send_goal_async(self._goal)

        if blocking:
            while not future.done():
                self.node.get_logger().debug(
                    "Waiting for goal answer...", throttle_duration_sec=1.0
                )

            goal_handle = future.result()

            future = goal_handle.get_result_async()
            while not future.done():
                self.node.get_logger().debug(
                    "Waiting for goal result...", throttle_duration_sec=1.0
                )

            self.node.get_logger().debug(f"Goal result: {future.result()}")
            return future.result()
