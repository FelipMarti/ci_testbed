"""Brightness-threshold blob detector used by the vision pipeline."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ci_testbed.errors import ConfigurationError, TargetNotFound
from ci_testbed.geometry import Pixel
from ci_testbed.sensors.interface import FloatArray


@dataclass(frozen=True)
class BlobDetector:
    """Finds the intensity-weighted centroid of the brightest blob."""

    threshold: float = 0.5
    min_pixels: int = 3

    def __post_init__(self) -> None:
        if not 0.0 <= self.threshold <= 1.0:
            raise ConfigurationError("threshold must be between 0 and 1")
        if self.min_pixels < 1:
            raise ConfigurationError("min_pixels must be positive")

    def detect(self, image: FloatArray) -> Pixel:
        if image.ndim != 2:
            raise ValueError("expected a 2D grayscale image")

        rows, columns = np.nonzero(image >= self.threshold)
        if rows.size < self.min_pixels:
            raise TargetNotFound(f"only {rows.size} pixels passed threshold {self.threshold:.2f}")

        weights = image[rows, columns]
        total_weight = float(weights.sum())
        if total_weight <= 0:
            raise TargetNotFound("blob has no positive intensity")

        u = float((columns * weights).sum() / total_weight)
        v = float((rows * weights).sum() / total_weight)
        return Pixel(u=u, v=v)
