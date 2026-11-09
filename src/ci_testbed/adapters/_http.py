from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from typing import cast
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class HttpRequestError(Exception):
    """A transport or payload failure returned by an HTTP mock service."""

    def __init__(
        self,
        message: str,
        *,
        status: int | None = None,
        code: str | None = None,
    ) -> None:
        super().__init__(message)
        self.status = status
        self.code = code


def _error_payload(raw: bytes, fallback: str) -> tuple[str | None, str]:
    try:
        payload: object = json.loads(raw)
    except (json.JSONDecodeError, UnicodeDecodeError):
        return None, fallback

    if not isinstance(payload, dict):
        return None, fallback

    detail = payload.get("detail")
    if isinstance(detail, dict):
        code = detail.get("error")
        message = detail.get("message")
        if isinstance(code, str) and isinstance(message, str):
            return code, message
    if isinstance(detail, str):
        return None, detail
    return None, fallback


class HttpClient:
    """Tiny JSON client built on the standard library."""

    def __init__(self, base_url: str, *, timeout: float = 5.0) -> None:
        if not base_url:
            raise ValueError("base_url must not be empty")
        if timeout <= 0:
            raise ValueError("timeout must be positive")
        self._base_url = base_url.rstrip("/")
        self._timeout = timeout

    def get(self, path: str) -> object:
        return self._request("GET", path)

    def post(self, path: str, payload: Mapping[str, object] | None = None) -> object:
        return self._request("POST", path, payload)

    def _request(
        self,
        method: str,
        path: str,
        payload: Mapping[str, object] | None = None,
    ) -> object:
        body = None if payload is None else json.dumps(payload).encode("utf-8")
        request = Request(
            f"{self._base_url}{path}",
            data=body,
            method=method,
            headers={"Accept": "application/json", "Content-Type": "application/json"},
        )

        try:
            with urlopen(request, timeout=self._timeout) as response:
                raw = response.read()
        except HTTPError as exc:
            try:
                code, message = _error_payload(exc.read(), str(exc))
            finally:
                exc.close()
            raise HttpRequestError(message, status=exc.code, code=code) from exc
        except (URLError, TimeoutError, OSError) as exc:
            raise HttpRequestError(f"{method} {path} failed: {exc}") from exc

        try:
            return json.loads(raw)
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            raise HttpRequestError(f"{method} {path} returned invalid JSON") from exc


def as_mapping(value: object, context: str) -> Mapping[str, object]:
    if not isinstance(value, dict):
        raise HttpRequestError(f"{context}: expected a JSON object")
    return cast("Mapping[str, object]", value)


def as_sequence(value: object, context: str) -> Sequence[object]:
    if not isinstance(value, (list, tuple)):
        raise HttpRequestError(f"{context}: expected a JSON array")
    return cast("Sequence[object]", value)


def as_float(value: object, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise HttpRequestError(f"{context}: expected a number")
    return float(value)


def as_int(value: object, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise HttpRequestError(f"{context}: expected an integer")
    return value


def as_bool(value: object, context: str) -> bool:
    if not isinstance(value, bool):
        raise HttpRequestError(f"{context}: expected a boolean")
    return value


def required(mapping: Mapping[str, object], key: str, context: str) -> object:
    try:
        return mapping[key]
    except KeyError as exc:
        raise HttpRequestError(f"{context}: missing key {key!r}") from exc
