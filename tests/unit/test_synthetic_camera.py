from __future__ import annotations

import numpy as np
import pytest

from ci_testbed.clock import FakeClock
from ci_testbed.config import CameraConfig
from ci_testbed.errors import CameraFault, TargetNotFound
from ci_testbed.geometry import Vec2
from ci_testbed.perception.blob_detector import BlobDetector
from ci_testbed.sensors.synthetic_camera import SyntheticCamera

pytestmark = pytest.mark.unit


def test_capture_returns_expected_frame(camera: SyntheticCamera) -> None:
    frame = camera.capture()

    assert frame.image.shape == (120, 160)
    assert frame.image.dtype == np.float64
    assert frame.frame_id == 0
    assert frame.timestamp == 0.0
    assert camera.frames_captured == 1


def test_target_is_rendered_at_expected_pixel(camera: SyntheticCamera) -> None:
    frame = camera.capture()
    expected = camera.calibration.world_to_pixel(camera.target_world)
    pixel = BlobDetector().detect(frame.image)

    assert pixel.u == pytest.approx(expected.u, abs=0.5)
    assert pixel.v == pytest.approx(expected.v, abs=0.5)
    assert frame.image.max() == pytest.approx(1.0)


def test_noise_is_deterministic_for_same_seed() -> None:
    config = CameraConfig(noise_std=0.05, seed=123)
    target = Vec2(0.3, 0.3)
    first = SyntheticCamera(config, FakeClock(), target_world=target)
    second = SyntheticCamera(config, FakeClock(), target_world=target)
    third = SyntheticCamera(
        CameraConfig(noise_std=0.05, seed=999), FakeClock(), target_world=target
    )

    assert np.array_equal(first.capture().image, second.capture().image)
    assert not np.array_equal(first.capture().image, third.capture().image)


def test_exposure_scales_brightness() -> None:
    dim = SyntheticCamera(CameraConfig(exposure=0.25, target_intensity=1.0), FakeClock())
    bright = SyntheticCamera(CameraConfig(exposure=1.0, target_intensity=1.0), FakeClock())

    assert dim.capture().image.max() == pytest.approx(0.25)
    assert bright.capture().image.max() == pytest.approx(1.0)


def test_injected_camera_fault(camera: SyntheticCamera) -> None:
    camera.fail_next(1)

    with pytest.raises(CameraFault):
        camera.capture()

    assert camera.frames_captured == 0
    frame = camera.capture()
    assert frame.frame_id == 0


def test_negative_fault_count_is_rejected(camera: SyntheticCamera) -> None:
    with pytest.raises(ValueError):
        camera.fail_next(-1)


def test_target_outside_image_is_blank(camera: SyntheticCamera) -> None:
    camera.set_target_world(Vec2(10.0, 10.0))

    frame = camera.capture()

    assert frame.image.max() == 0.0
    with pytest.raises(TargetNotFound):
        BlobDetector().detect(frame.image)


def test_last_frame_is_cached(camera: SyntheticCamera) -> None:
    assert camera.last_frame is None
    frame = camera.capture()
    assert camera.last_frame is frame
