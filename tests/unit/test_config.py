from __future__ import annotations

import math

import pytest

from ci_testbed.config import (
    ArmConfig,
    CameraConfig,
    CellConfig,
    ControllerConfig,
    RangeSensorConfig,
)
from ci_testbed.errors import ConfigurationError
from ci_testbed.geometry import Vec2

pytestmark = pytest.mark.unit


def test_default_config_is_valid() -> None:
    config = CellConfig()

    assert config.camera.calibration().contains(config.target_world)
    assert config.target_world.norm() < config.arm.reach


def test_camera_frame_period() -> None:
    assert CameraConfig(fps=50.0).frame_period == pytest.approx(0.02)


@pytest.mark.parametrize(
    "options",
    [
        {"width": 0},
        {"height": 0},
        {"fps": 0},
        {"exposure": -1.0},
        {"noise_std": -0.1},
        {"target_radius_px": 0},
        {"meters_per_pixel": 0.0},
    ],
)
def test_invalid_camera_configs(options: dict[str, float]) -> None:
    with pytest.raises(ConfigurationError):
        CellConfig(camera=CameraConfig(**options))  # type: ignore[arg-type]


@pytest.mark.parametrize(
    "options",
    [
        {"link1": 0.0},
        {"link2": -1.0},
        {"joint_min": (1.0, -math.pi), "joint_max": (-1.0, math.pi)},
        {
            "joint_min": (0.0, -math.pi),
            "joint_max": (1.0, math.pi),
            "home_joints": (-1.0, 0.0),
        },
        {"max_joint_speed": 0.0},
    ],
)
def test_invalid_arm_configs(options: dict[str, object]) -> None:
    with pytest.raises(ConfigurationError):
        CellConfig(arm=ArmConfig(**options))  # type: ignore[arg-type]


@pytest.mark.parametrize(
    "options",
    [
        {"max_range": 0.0},
        {"noise_std": -1.0},
    ],
)
def test_invalid_range_configs(options: dict[str, float]) -> None:
    with pytest.raises(ConfigurationError):
        CellConfig(range_sensor=RangeSensorConfig(**options))  # type: ignore[arg-type]


@pytest.mark.parametrize(
    "options",
    [
        {"detection_retries": -1},
        {"position_tolerance": 0.0},
        {"retry_backoff": -1.0},
        {"safety_clearance": -0.1},
    ],
)
def test_invalid_controller_configs(options: dict[str, float]) -> None:
    with pytest.raises(ConfigurationError):
        CellConfig(controller=ControllerConfig(**options))  # type: ignore[arg-type]


def test_target_must_be_visible() -> None:
    with pytest.raises(ConfigurationError, match="visible"):
        CellConfig(target_world=Vec2(10.0, 10.0))


def test_target_must_be_reachable() -> None:
    with pytest.raises(ConfigurationError, match="workspace|visible"):
        CellConfig(target_world=Vec2(0.5, 0.43))
