"""Callback frequency monitor with diagnostics publishing for ROS2 nodes."""

import time
from collections import deque
from functools import wraps
from typing import Callable, Dict

import numpy as np
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node


class CallbackData:
    """Data container for individual callback monitoring."""

    def __init__(self, name: str, stale_threshold: float = 2.0, window_size: int = 50):
        """Initialize callback data."""
        self.name = name
        self.stale_threshold = stale_threshold
        self.timestamps = deque(maxlen=window_size)
        self.last_callback_time = None
        self.callback_count = 0

    @property
    def is_stale(self) -> bool:
        """Check if callback is stale."""
        if self.last_callback_time is None:
            return True
        return (time.time() - self.last_callback_time) > self.stale_threshold

    @property
    def frequency(self) -> float:
        """Current callback frequency."""
        if len(self.timestamps) < 2:
            return 0.0
        time_span = self.timestamps[-1] - self.timestamps[0]
        return (len(self.timestamps) - 1) / time_span if time_span > 0 else 0.0

    @property
    def mean_interval(self) -> float:
        """Mean time between callbacks."""
        freq = self.frequency
        return 1.0 / freq if freq > 0 else 0.0

    @property
    def interval_stddev(self) -> float:
        """Standard deviation of time intervals between callbacks."""
        if len(self.timestamps) < 3:
            return 0.0

        # Calculate intervals between consecutive timestamps
        intervals = []
        for i in range(1, len(self.timestamps)):
            intervals.append(self.timestamps[i] - self.timestamps[i - 1])

        # Return standard deviation of intervals
        return float(np.std(intervals)) if intervals else 0.0

    def update(self):
        """Update callback data with current timestamp."""
        current_time = time.time()
        self.timestamps.append(current_time)
        self.last_callback_time = current_time
        self.callback_count += 1


class CallbackMonitor:
    """Multi-callback frequency monitor with single diagnostics publisher."""

    def __init__(self, node: Node, stale_threshold: float = 2.0, window_size: int = 50):
        """Initialize the callback monitor.

        Args:
            node: ROS2 node for publishing diagnostics
            stale_threshold: Time (seconds) after which callback is considered stale
            window_size: Number of samples to keep for frequency calculation
        """
        self.node = node
        self.stale_threshold = stale_threshold
        self.window_size = window_size

        self.callbacks: Dict[str, CallbackData] = {}

        self.diag_pub = node.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.timer = node.create_timer(1.0, self._publish_diagnostics)

    def monitor(self, name: str, func: Callable) -> Callable:
        """Decorator to monitor callback frequency.

        Args:
            name: Name identifier for the callback
            func: Function to monitor (when used as decorator)
        """

        def decorator(f: Callable) -> Callable:
            # Initialize callback data if not exists
            if name not in self.callbacks:
                self.callbacks[name] = CallbackData(
                    name=name, stale_threshold=self.stale_threshold, window_size=self.window_size
                )

            @wraps(f)
            def wrapper(*args: tuple, **kwargs: dict):  # noqa: ANN202
                self.callbacks[name].update()
                return f(*args, **kwargs)

            return wrapper

        return decorator(func)

    def get_callback_data(self, name: str) -> CallbackData:
        """Get callback data for a specific callback."""
        if name not in self.callbacks.keys():
            raise ValueError(f"Callback '{name}' is not being monitored.")
        return self.callbacks[name]

    def _publish_diagnostics(self):
        """Publish diagnostics for all monitored callbacks at 1 Hz."""
        if not self.callbacks:
            return

        statuses = []
        for callback_data in self.callbacks.values():
            if callback_data.is_stale:
                level = DiagnosticStatus.STALE
                message = f"Callback stale ({time.time() - (callback_data.last_callback_time or 0):.4f}s ago)"
            else:
                level = DiagnosticStatus.OK
                message = "Callback active"

            status = DiagnosticStatus(
                level=level,
                name=f"{callback_data.name.capitalize()} Monitor",
                message=message,
                values=[
                    KeyValue(key="frequency_hz", value=f"{callback_data.frequency:.4f}"),
                    KeyValue(
                        key="mean_interval_ms", value=f"{callback_data.mean_interval * 1000:.1f}"
                    ),
                    KeyValue(
                        key="interval_stddev_ms",
                        value=f"{callback_data.interval_stddev * 1000:.1f}",
                    ),
                ],
            )
            statuses.append(status)

        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.node.get_clock().now().to_msg()
        diag_array.status = statuses
        self.diag_pub.publish(diag_array)
