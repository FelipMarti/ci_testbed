"""Synthetic top-down camera with deterministic noise and fault injection."""

from __future__ import annotations

import numpy as np

from ci_testbed.clock import Clock
from ci_testbed.config import CameraConfig
from ci_testbed.errors import CameraFault
from ci_testbed.geometry import CameraCalibration, Pixel, Vec2
from ci_testbed.sensors.interface import Frame


class SyntheticCamera:
    """Renders a bright target disk into a configurable image."""

    def __init__(
        self,
        config: CameraConfig,
        clock: Clock,
        *,
        calibration: CameraCalibration | None = None,
        target_world: Vec2 | None = None,
    ) -> None:
        self._config = config
        self._clock = clock
        self._calibration = calibration or config.calibration()
        self._target_world = target_world or self._calibration.pixel_to_world(
            Pixel((config.width - 1) / 2.0, (config.height - 1) / 2.0)
        )
        self._rng = np.random.default_rng(config.seed)
        self._frame_id = 0
        self._faults_remaining = 0
        self._last_frame: Frame | None = None

    @property
    def calibration(self) -> CameraCalibration:
        return self._calibration

    @property
    def target_world(self) -> Vec2:
        return self._target_world

    @property
    def frames_captured(self) -> int:
        return self._frame_id

    @property
    def last_frame(self) -> Frame | None:
        return self._last_frame

    def set_target_world(self, target: Vec2) -> None:
        self._target_world = target

    def fail_next(self, count: int = 1) -> None:
        """Inject ``count`` consecutive capture failures."""

        if count < 0:
            raise ValueError("count must be non-negative")
        self._faults_remaining += count

    def capture(self) -> Frame:
        if self._faults_remaining > 0:
            self._faults_remaining -= 1
            raise CameraFault("injected camera failure")

        timestamp = self._clock.now()
        image = np.full(
            (self._config.height, self._config.width),
            self._config.background_intensity * self._config.exposure,
            dtype=np.float64,
        )

        target_pixel = self._calibration.world_to_pixel(self._target_world)
        if self._pixel_in_bounds(target_pixel):
            u_coords, v_coords = np.meshgrid(
                np.arange(self._config.width), np.arange(self._config.height)
            )
            radius_squared = self._config.target_radius_px**2
            mask = (u_coords - target_pixel.u) ** 2 + (
                v_coords - target_pixel.v
            ) ** 2 <= radius_squared
            image[mask] = self._config.target_intensity * self._config.exposure

        if self._config.noise_std > 0:
            image += self._rng.normal(0.0, self._config.noise_std, size=image.shape)
        np.clip(image, 0.0, 1.0, out=image)

        frame = Frame(image=image, timestamp=timestamp, frame_id=self._frame_id)
        self._frame_id += 1
        self._last_frame = frame
        return frame

    def _pixel_in_bounds(self, pixel: Pixel) -> bool:
        return 0 <= pixel.u <= self._config.width - 1 and 0 <= pixel.v <= self._config.height - 1
