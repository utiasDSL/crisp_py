"""A sliding buffer implementation using deque from the collections module."""

from collections import deque
from typing import Any


class SlidingBuffer:
    """A fixed-size sliding buffer that retains the most recent elements."""

    def __init__(
        self,
        size: int,
        fill_buffer: bool = True,
        fill_value: Any = 0.0,
        buffer_type: type = float,
    ) -> None:
        """Initialize the sliding buffer with a given size."""
        self.buffer = deque(maxlen=size)
        self.buffer_type = buffer_type
        if fill_buffer:
            for _ in range(size):
                self.buffer.append(fill_value)

    def add(self, value: Any) -> None:
        """Add a new value to the buffer."""
        if not isinstance(value, self.buffer_type):
            raise TypeError(f"Value must be of type {self.buffer_type.__name__}")
        self.buffer.append(value)

    def get(self) -> list:
        """Get a list of the current elements in the buffer."""
        return list(self.buffer)
