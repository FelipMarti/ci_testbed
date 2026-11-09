"""Sensor abstractions shared by simulators, HTTP mocks, and ROS2 adapters."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.float64]


@dataclass(frozen=True)
class Frame:
    """A single camera frame with a monotonic id and capture timestamp."""

    image: FloatArray
    timestamp: float
    frame_id: int


@runtime_checkable
class Camera(Protocol):
    """Camera interface used by perception and the controller."""

    def capture(self) -> Frame:
        """Return the next frame or raise ``CameraFault``."""


@runtime_checkable
class RangeSensor(Protocol):
    """One-dimensional range sensor used for the safety check."""

    def read_distance(self) -> float:
        """Return the nearest obstacle distance in metres."""
