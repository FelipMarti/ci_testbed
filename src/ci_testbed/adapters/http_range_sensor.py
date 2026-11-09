"""Range sensor adapter for the containerised lidar mock."""

from __future__ import annotations

from ci_testbed.adapters._http import (
    HttpClient,
    HttpRequestError,
    as_float,
    as_mapping,
    required,
)
from ci_testbed.errors import SensorDropout


class HttpRangeSensor:
    """Range sensor implementation backed by ``lidar-mock`` over HTTP."""

    def __init__(self, base_url: str, *, timeout: float = 5.0) -> None:
        self._client = HttpClient(base_url, timeout=timeout)
        self._last_reading: float | None = None

    @property
    def last_reading(self) -> float | None:
        return self._last_reading

    def read_distance(self) -> float:
        payload = self._call("get", "/distance")
        try:
            data = as_mapping(payload, "distance")
            reading = as_float(required(data, "distance", "distance"), "distance.distance")
        except HttpRequestError as exc:
            raise SensorDropout(str(exc)) from exc
        self._last_reading = reading
        return reading

    def set_distance(self, distance: float) -> None:
        if distance < 0:
            raise ValueError("distance must be non-negative")
        self._call("post", "/distance", {"distance": distance})

    def drop_next(self, count: int = 1) -> None:
        if count < 0:
            raise ValueError("count must be non-negative")
        self._call("post", "/faults", {"count": count})

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
            if exc.status == 503 and exc.code == "sensor_dropout":
                raise SensorDropout(str(exc)) from exc
            raise SensorDropout(f"lidar service request failed: {exc}") from exc
