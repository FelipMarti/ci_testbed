"""Validated configuration objects for the simulated robotic cell."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from ci_testbed.errors import ConfigurationError
from ci_testbed.geometry import CameraCalibration, Vec2


@dataclass(frozen=True)
class CameraConfig:
    width: int = 160
    height: int = 120
    fps: float = 30.0
    exposure: float = 1.0
    noise_std: float = 0.0
    seed: int = 42
    target_radius_px: int = 6
    target_intensity: float = 1.0
    background_intensity: float = 0.0
    meters_per_pixel: float = 0.002
    world_origin: Vec2 = Vec2(0.2, 0.2)

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ConfigurationError("camera width and height must be positive")
        if self.fps <= 0:
            raise ConfigurationError("camera fps must be positive")
        if self.exposure < 0:
            raise ConfigurationError("camera exposure must be non-negative")
        if self.noise_std < 0:
            raise ConfigurationError("camera noise_std must be non-negative")
        if self.target_radius_px <= 0:
            raise ConfigurationError("target_radius_px must be positive")
        if self.meters_per_pixel <= 0:
            raise ConfigurationError("meters_per_pixel must be positive")

    @property
    def frame_period(self) -> float:
        return 1.0 / self.fps

    def calibration(self) -> CameraCalibration:
        return CameraCalibration(
            width=self.width,
            height=self.height,
            meters_per_pixel=self.meters_per_pixel,
            world_origin=self.world_origin,
        )


@dataclass(frozen=True)
class ArmConfig:
    link1: float = 0.35
    link2: float = 0.25
    joint_min: tuple[float, float] = (-math.pi, -math.pi)
    joint_max: tuple[float, float] = (math.pi, math.pi)
    home_joints: tuple[float, float] = (math.pi / 4.0, -math.pi / 4.0)
    max_joint_speed: float = 1.0

    def __post_init__(self) -> None:
        if self.link1 <= 0 or self.link2 <= 0:
            raise ConfigurationError("arm link lengths must be positive")
        if len(self.joint_min) != 2 or len(self.joint_max) != 2:
            raise ConfigurationError("joint limits must contain exactly two values")
        if any(lower >= upper for lower, upper in zip(self.joint_min, self.joint_max, strict=True)):
            raise ConfigurationError("each joint_min must be smaller than joint_max")
        for joint, (lower, upper) in enumerate(zip(self.joint_min, self.joint_max, strict=True)):
            home = self.home_joints[joint]
            if not lower <= home <= upper:
                raise ConfigurationError(f"home position is outside joint {joint} limits")
        if self.max_joint_speed <= 0:
            raise ConfigurationError("max_joint_speed must be positive")

    @property
    def reach(self) -> float:
        return self.link1 + self.link2


@dataclass(frozen=True)
class RangeSensorConfig:
    max_range: float = 2.0
    noise_std: float = 0.002
    seed: int = 7

    def __post_init__(self) -> None:
        if self.max_range <= 0:
            raise ConfigurationError("range sensor max_range must be positive")
        if self.noise_std < 0:
            raise ConfigurationError("range sensor noise_std must be non-negative")


@dataclass(frozen=True)
class ControllerConfig:
    detection_retries: int = 3
    position_tolerance: float = 0.01
    retry_backoff: float = 0.01
    safety_clearance: float = 0.15

    def __post_init__(self) -> None:
        if self.detection_retries < 0:
            raise ConfigurationError("detection_retries must be non-negative")
        if self.position_tolerance <= 0:
            raise ConfigurationError("position_tolerance must be positive")
        if self.retry_backoff < 0:
            raise ConfigurationError("retry_backoff must be non-negative")
        if self.safety_clearance < 0:
            raise ConfigurationError("safety_clearance must be non-negative")


@dataclass(frozen=True)
class CellConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    arm: ArmConfig = field(default_factory=ArmConfig)
    range_sensor: RangeSensorConfig = field(default_factory=RangeSensorConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    target_world: Vec2 = Vec2(0.35, 0.30)

    def __post_init__(self) -> None:
        if not self.camera.calibration().contains(self.target_world):
            raise ConfigurationError("target_world must be visible to the camera")
        if self.target_world.norm() > self.arm.reach:
            raise ConfigurationError("target_world must be inside the arm workspace")
