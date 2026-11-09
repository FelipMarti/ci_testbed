"""Camera adapter for the containerised camera mock."""

from __future__ import annotations

import numpy as np

from ci_testbed.adapters._http import (
    HttpClient,
    HttpRequestError,
    as_float,
    as_int,
    as_mapping,
    required,
)
from ci_testbed.config import CameraConfig
from ci_testbed.errors import CameraFault
from ci_testbed.geometry import CameraCalibration, Vec2
from ci_testbed.sensors.interface import Frame


class HttpCamera:
    """Camera implementation backed by ``camera-mock`` over HTTP."""

    def __init__(
        self,
        base_url: str,
        *,
        calibration: CameraCalibration | None = None,
        target_world: Vec2 | None = None,
        timeout: float = 5.0,
    ) -> None:
        config = CameraConfig()
        self._client = HttpClient(base_url, timeout=timeout)
        self._calibration = calibration or config.calibration()
        self._target_world = target_world or Vec2(0.35, 0.30)
        self._last_frame: Frame | None = None
        self._frames_captured = 0

    @property
    def calibration(self) -> CameraCalibration:
        return self._calibration

    @property
    def target_world(self) -> Vec2:
        return self._target_world

    @property
    def frames_captured(self) -> int:
        return self._frames_captured

    @property
    def last_frame(self) -> Frame | None:
        return self._last_frame

    def set_target_world(self, target: Vec2) -> None:
        self._call("post", "/target", {"x": target.x, "y": target.y})
        self._target_world = target

    def fail_next(self, count: int = 1) -> None:
        if count < 0:
            raise ValueError("count must be non-negative")
        self._call("post", "/faults", {"count": count})

    def capture(self) -> Frame:
        payload = self._call("get", "/frame")
        try:
            data = as_mapping(payload, "frame")
            image = np.asarray(required(data, "image", "frame"), dtype=np.float64)
            if image.ndim != 2:
                raise HttpRequestError("frame: image must be two-dimensional")
            frame = Frame(
                image=image,
                timestamp=as_float(required(data, "timestamp", "frame"), "frame.timestamp"),
                frame_id=as_int(required(data, "frame_id", "frame"), "frame.frame_id"),
            )
        except HttpRequestError as exc:
            raise CameraFault(str(exc)) from exc

        self._last_frame = frame
        self._frames_captured += 1
        return frame

    def _call(
        self,
        method: str,
        path: str,
        payload: dict[str, object] | None = None,
    ) -> object:
        try:
            if method == "get":
                return self._client.get(path)
            return self._client.post(path, payload)
        except HttpRequestError as exc:
            if exc.status == 503 and exc.code == "camera_fault":
                raise CameraFault(str(exc)) from exc
            raise CameraFault(f"camera service request failed: {exc}") from exc
