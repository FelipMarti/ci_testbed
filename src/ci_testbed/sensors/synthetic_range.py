"""Synthetic range sensor with deterministic noise and dropout injection."""

from __future__ import annotations

import numpy as np

from ci_testbed.clock import Clock
from ci_testbed.config import RangeSensorConfig
from ci_testbed.errors import SensorDropout


class SyntheticRangeSensor:
    """Reports a configurable obstacle distance with optional Gaussian noise."""

    def __init__(self, config: RangeSensorConfig, clock: Clock) -> None:
        self._config = config
        self._clock = clock
        self._rng = np.random.default_rng(config.seed)
        self._distance = config.max_range
        self._dropouts_remaining = 0
        self._last_reading: float | None = None

    @property
    def last_reading(self) -> float | None:
        return self._last_reading

    def set_distance(self, distance: float) -> None:
        if distance < 0:
            raise ValueError("distance must be non-negative")
        if distance > self._config.max_range:
            raise ValueError("distance cannot exceed max_range")
        self._distance = distance

    def drop_next(self, count: int = 1) -> None:
        """Inject ``count`` consecutive sensor dropouts."""

        if count < 0:
            raise ValueError("count must be non-negative")
        self._dropouts_remaining += count

    def read_distance(self) -> float:
        if self._dropouts_remaining > 0:
            self._dropouts_remaining -= 1
            raise SensorDropout("injected range sensor dropout")

        value = self._distance
        if self._config.noise_std > 0:
            value += float(self._rng.normal(0.0, self._config.noise_std))
        reading = max(0.0, min(self._config.max_range, value))
        self._last_reading = reading
        return reading

    @property
    def timestamp(self) -> float:
        return self._clock.now()
