"""Utility functions for geometry operations."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class Pose:
    """Compact representation of an SE3 object."""

    position: np.ndarray
    orientation: Rotation

    def copy(self) -> "Pose":
        """Create a copy of this pose."""
        return Pose(self.position.copy(), Rotation.from_quat(self.orientation.as_quat()))

    def __str__(self) -> str:
        """Return a string representation of a Pose."""
        return f"Pos: {np.array2string(self.position, suppress_small=True, precision=2, floatmode='fixed')},\n Orientation: {np.array2string(self.orientation.as_matrix(), suppress_small=True, precision=2, floatmode='fixed')}"

    def __sub__(self, other: "Pose") -> "Pose":
        """Subtract another pose from this pose, i.e. compute the relative pose."""
        return Pose(
            self.position - other.position,
            self.orientation * other.orientation.inv(),
        )

    def __add__(self, other: "Pose") -> "Pose":
        """Add another pose to this pose, i.e. add a relative pose."""
        return Pose(
            self.position + other.position,
            other.orientation * self.orientation,
        )
