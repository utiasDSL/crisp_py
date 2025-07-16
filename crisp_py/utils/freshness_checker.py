"""Utility class for checking data freshness and logging warnings."""

import time
from typing import Optional

from rclpy.node import Node


class FreshnessChecker:
    """Utility class to check data freshness and log throttled warnings."""

    def __init__(
        self, node: Node, data_name: str, max_delay: float, throttle_duration: float = 5.0
    ):
        """Initialize the freshness checker.

        Args:
            node: ROS2 node for logging
            data_name: Name of the data being checked (for logging)
            max_delay: Maximum acceptable delay in seconds
            throttle_duration: Throttle duration for warning messages in seconds
        """
        self.node = node
        self.data_name = data_name
        self.max_delay = max_delay
        self.throttle_duration = throttle_duration
        self._last_timestamp: Optional[float] = None

    def update_timestamp(self) -> None:
        """Update the timestamp when new data is received."""
        self._last_timestamp = time.time()

    def check_freshness(self) -> None:
        """Check if the data is fresh and log a warning if stale."""
        if self._last_timestamp is None:
            return

        current_time = time.time()
        data_age = current_time - self._last_timestamp

        if data_age > self.max_delay:
            self.node.get_logger().warning(
                f"{self.data_name} data is stale. "
                f"Last data received {data_age:.2f}s ago, "
                f"exceeds max delay of {self.max_delay:.2f}s",
                throttle_duration_sec=self.throttle_duration,
            )

