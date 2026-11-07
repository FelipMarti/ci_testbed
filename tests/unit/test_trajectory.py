from __future__ import annotations

import pytest

from ci_testbed.geometry import Vec2
from ci_testbed.robot.trajectory import (
    interpolate_cartesian,
    interpolate_joints,
    joint_duration,
)

pytestmark = pytest.mark.unit


def test_interpolate_joints_includes_endpoints() -> None:
    waypoints = interpolate_joints((0.0, 0.0), (1.0, 2.0), 11)

    assert len(waypoints) == 11
    assert waypoints[0] == (0.0, 0.0)
    assert waypoints[-1] == (1.0, 2.0)
    assert waypoints[5] == pytest.approx((0.5, 1.0))


def test_interpolate_cartesian_includes_endpoints() -> None:
    waypoints = interpolate_cartesian(Vec2(0.0, 0.0), Vec2(1.0, 2.0), 3)

    assert waypoints == (Vec2(0.0, 0.0), Vec2(0.5, 1.0), Vec2(1.0, 2.0))


@pytest.mark.parametrize("steps", [0, 1, -1])
def test_interpolation_requires_at_least_two_steps(steps: int) -> None:
    with pytest.raises(ValueError):
        interpolate_joints((0.0, 0.0), (1.0, 1.0), steps)
    with pytest.raises(ValueError):
        interpolate_cartesian(Vec2(0.0, 0.0), Vec2(1.0, 1.0), steps)


def test_joint_duration_uses_slowest_joint() -> None:
    assert joint_duration((0.0, 0.0), (2.0, 1.0), 2.0) == pytest.approx(1.0)
    assert joint_duration((1.0, 1.0), (1.0, 1.0), 2.0) == 0.0


@pytest.mark.parametrize("speed", [0.0, -1.0])
def test_joint_duration_rejects_invalid_speed(speed: float) -> None:
    with pytest.raises(ValueError):
        joint_duration((0.0, 0.0), (1.0, 1.0), speed)
