from __future__ import annotations

import math

import pytest

from ci_testbed.geometry import (
    CameraCalibration,
    Pixel,
    Vec2,
    clamp,
    normalize_angle,
)

pytestmark = pytest.mark.unit


def test_vec2_arithmetic() -> None:
    first = Vec2(1.0, 2.0)
    second = Vec2(3.0, 5.0)

    assert first + second == Vec2(4.0, 7.0)
    assert second - first == Vec2(2.0, 3.0)
    assert first * 2 == Vec2(2.0, 4.0)
    assert 2 * first == Vec2(2.0, 4.0)
    assert first / 2 == Vec2(0.5, 1.0)
    assert -first == Vec2(-1.0, -2.0)


def test_vec2_norm_and_distance() -> None:
    assert Vec2(3.0, 4.0).norm() == pytest.approx(5.0)
    assert Vec2(0.0, 0.0).distance_to(Vec2(3.0, 4.0)) == pytest.approx(5.0)


def test_vec2_division_by_zero_is_rejected() -> None:
    with pytest.raises(ZeroDivisionError):
        Vec2(1.0, 1.0) / 0.0


@pytest.mark.parametrize(
    "pixel",
    [Pixel(0.0, 0.0), Pixel(159.0, 119.0), Pixel(75.5, 69.5), Pixel(10.0, 100.0)],
)
def test_calibration_round_trip(pixel: Pixel) -> None:
    calibration = CameraCalibration(
        width=160,
        height=120,
        meters_per_pixel=0.002,
        world_origin=Vec2(0.2, 0.2),
    )

    world = calibration.pixel_to_world(pixel)
    recovered = calibration.world_to_pixel(world)

    assert recovered.u == pytest.approx(pixel.u)
    assert recovered.v == pytest.approx(pixel.v)


def test_pixel_v_axis_points_down_in_world() -> None:
    calibration = CameraCalibration(
        width=160,
        height=120,
        meters_per_pixel=0.002,
        world_origin=Vec2(0.2, 0.2),
    )

    top = calibration.pixel_to_world(Pixel(0.0, 0.0))
    bottom = calibration.pixel_to_world(Pixel(0.0, 119.0))

    assert top.y > bottom.y


def test_calibration_contains() -> None:
    calibration = CameraCalibration(
        width=10,
        height=10,
        meters_per_pixel=0.1,
        world_origin=Vec2(0.0, 0.0),
    )

    assert calibration.contains(Vec2(0.5, 0.5))
    assert not calibration.contains(Vec2(2.0, 0.5))


@pytest.mark.parametrize(
    ("angle", "expected"),
    [
        (0.0, 0.0),
        (math.pi / 2.0, math.pi / 2.0),
        (math.pi, -math.pi),
        (3.0 * math.pi, -math.pi),
        (-math.pi / 2.0, -math.pi / 2.0),
    ],
)
def test_normalize_angle(angle: float, expected: float) -> None:
    assert normalize_angle(angle) == pytest.approx(expected)


def test_clamp() -> None:
    assert clamp(5.0, 0.0, 3.0) == 3.0
    assert clamp(-1.0, 0.0, 3.0) == 0.0
    assert clamp(2.0, 0.0, 3.0) == 2.0
    with pytest.raises(ValueError):
        clamp(1.0, 3.0, 0.0)
