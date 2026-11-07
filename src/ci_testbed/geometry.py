"""Geometry primitives and camera calibration helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ci_testbed.errors import ConfigurationError


@dataclass(frozen=True)
class Vec2:
    """A point or vector in the robot workspace, in metres."""

    x: float
    y: float

    def __add__(self, other: Vec2) -> Vec2:
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vec2) -> Vec2:
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> Vec2:
        return Vec2(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> Vec2:
        return self * scalar

    def __truediv__(self, scalar: float) -> Vec2:
        if scalar == 0:
            raise ZeroDivisionError("cannot divide a vector by zero")
        return Vec2(self.x / scalar, self.y / scalar)

    def __neg__(self) -> Vec2:
        return Vec2(-self.x, -self.y)

    def norm(self) -> float:
        return math.hypot(self.x, self.y)

    def distance_to(self, other: Vec2) -> float:
        return (self - other).norm()


@dataclass(frozen=True)
class Pose2D:
    """Planar pose of the tool centre point."""

    position: Vec2
    theta: float = 0.0


@dataclass(frozen=True)
class Pixel:
    """Image coordinate. ``u`` grows right, ``v`` grows down."""

    u: float
    v: float


@dataclass(frozen=True)
class CameraCalibration:
    """Maps between image pixels and a top-down world plane.

    ``world_origin`` is the world position of pixel ``(0, height - 1)``.
    """

    width: int
    height: int
    meters_per_pixel: float
    world_origin: Vec2 = Vec2(0.0, 0.0)

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ConfigurationError("camera dimensions must be positive")
        if self.meters_per_pixel <= 0:
            raise ConfigurationError("meters_per_pixel must be positive")

    def pixel_to_world(self, pixel: Pixel) -> Vec2:
        return Vec2(
            self.world_origin.x + pixel.u * self.meters_per_pixel,
            self.world_origin.y + (self.height - 1 - pixel.v) * self.meters_per_pixel,
        )

    def world_to_pixel(self, world: Vec2) -> Pixel:
        return Pixel(
            (world.x - self.world_origin.x) / self.meters_per_pixel,
            (self.height - 1) - (world.y - self.world_origin.y) / self.meters_per_pixel,
        )

    def contains(self, world: Vec2) -> bool:
        pixel = self.world_to_pixel(world)
        return 0 <= pixel.u <= self.width - 1 and 0 <= pixel.v <= self.height - 1


def normalize_angle(angle: float) -> float:
    """Wrap an angle to the half-open interval ``[-pi, pi)``."""

    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp ``value`` into ``[lower, upper]``."""

    if lower > upper:
        raise ValueError("lower must be <= upper")
    return max(lower, min(upper, value))
