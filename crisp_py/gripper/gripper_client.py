"""Base class for gripper client that provides a common interface for all grippers."""

from abc import ABC, abstractmethod


class GripperClient(ABC):
    """Interface for gripper client."""

    @property
    @abstractmethod
    def min_width(self) -> float:
        """Returns the minimum width of the gripper."""
        ...

    @property
    @abstractmethod
    def max_width(self) -> float:
        """Returns the minimum width of the gripper."""
        ...

    @property
    @abstractmethod
    def width(self) -> float | None:
        """Returns the current width of the gripper or None if not initialized."""
        ...

    @abstractmethod
    def is_ready(self) -> bool:
        """Returns True if the gripper is fully ready to operate."""
        ...

    @abstractmethod
    def wait_until_ready(self, timeout_sec: float = 5.0):
        """Waits until the gripper is fully ready to operate."""
        ...

    # === Implemented functions ===

    def grasp(
        self,
        width: float,
        speed: float = 0.1,
        force: float = 50.0,
        epsilon_outer: float = 0.08,
        epsilon_inner: float = 0.01,
        block: bool = False,
    ):
        """Grasp with the gripper and does not block.
        Args:
            width (float): The width of the gripper.
            speed (float, optional): The speed of the gripper. Defaults to 0.1.
            force (float, optional): The force of the gripper. Defaults to 50.0.
            epsilon_outer (float, optional): The outer epsilon of the gripper. Defaults to 0.08.
            epsilon_inner (float, optional): The inner epsilon of the gripper. Defaults to 0.01.
            block (bool, optional): Whether to block. Defaults to False.
        """
        ...

    def close(self, **grasp_kwargs):
        """Close the gripper.

        Args:
            **grasp_kwargs: Keyword arguments to pass to the grasp function. (check the grasp function for details)
        """
        self.grasp(width=self.min_width, **grasp_kwargs)

    def open(self, **grasp_kwargs):
        """Open the gripper.

        Args:
            **grasp_kwargs: Keyword arguments to pass to the grasp function. (check the grasp function for details)
        """
        self.grasp(width=self.max_width, **grasp_kwargs)

    def toggle(self, **grasp_kwargs):
        """Toggle the gripper between open and closed.

        Args:
            **grasp_kwargs: Keyword arguments to pass to the grasp function. (check the grasp function for details)
        """
        if self.is_open():
            self.close(**grasp_kwargs)
        else:
            self.open(**grasp_kwargs)

    def is_open(self, open_threshold: float = 0.07) -> bool:
        """Returns True if the gripper is open."""
        return self.width > open_threshold if self.width is not None else False
