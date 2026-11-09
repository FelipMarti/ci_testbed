"""Clock abstractions keep time-dependent tests deterministic."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Protocol


class Clock(Protocol):
    """Minimal clock interface used by the simulator."""

    def now(self) -> float:
        """Return the current time in seconds."""

    def sleep(self, seconds: float) -> None:
        """Advance time by ``seconds``."""


class SystemClock:
    """Wall-clock implementation used by real adapters and the CLI."""

    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)


@dataclass
class FakeClock:
    """Deterministic clock for tests and the simulated CLI."""

    _time: float = field(default=0.0)

    def now(self) -> float:
        return self._time

    def sleep(self, seconds: float) -> None:
        if seconds < 0:
            raise ValueError("cannot sleep for a negative duration")
        self._time += seconds

    def advance(self, seconds: float) -> None:
        self.sleep(seconds)
