from __future__ import annotations

from dataclasses import replace

import pytest

from ci_testbed.clock import FakeClock
from ci_testbed.config import CameraConfig
from ci_testbed.errors import TargetNotFound
from ci_testbed.geometry import CameraCalibration, Vec2
from ci_testbed.perception.blob_detector import BlobDetector
from ci_testbed.sensors.synthetic_camera import SyntheticCamera

pytestmark = pytest.mark.integration


@pytest.mark.parametrize(
    "target",
    [Vec2(0.30, 0.25), Vec2(0.35, 0.30), Vec2(0.45, 0.40), Vec2(0.22, 0.42)],
)
def test_camera_detector_calibration_pipeline(
    target: Vec2, calibration: CameraCalibration, fake_clock: FakeClock
) -> None:
    camera = SyntheticCamera(
        CameraConfig(), fake_clock, calibration=calibration, target_world=target
    )

    frame = camera.capture()
    pixel = BlobDetector().detect(frame.image)
    recovered = calibration.pixel_to_world(pixel)

    assert recovered.distance_to(target) <= calibration.meters_per_pixel


def test_pipeline_is_robust_to_moderate_noise(
    calibration: CameraCalibration, fake_clock: FakeClock
) -> None:
    target = Vec2(0.33, 0.27)
    camera = SyntheticCamera(
        CameraConfig(noise_std=0.02, seed=5),
        fake_clock,
        calibration=calibration,
        target_world=target,
    )

    frame = camera.capture()
    pixel = BlobDetector(threshold=0.6).detect(frame.image)
    recovered = calibration.pixel_to_world(pixel)

    assert recovered.distance_to(target) <= 2 * calibration.meters_per_pixel


def test_frame_timestamps_follow_the_clock(
    calibration: CameraCalibration, fake_clock: FakeClock
) -> None:
    camera = SyntheticCamera(CameraConfig(), fake_clock, calibration=calibration)

    first = camera.capture()
    fake_clock.advance(0.1)
    second = camera.capture()

    assert second.timestamp - first.timestamp == pytest.approx(0.1)
    assert second.frame_id == first.frame_id + 1


def test_config_with_zero_exposure_has_no_detection(
    calibration: CameraCalibration, fake_clock: FakeClock
) -> None:
    config = replace(CameraConfig(), exposure=0.0)
    camera = SyntheticCamera(config, fake_clock, calibration=calibration)
    frame = camera.capture()

    with pytest.raises(TargetNotFound, match="threshold"):
        BlobDetector().detect(frame.image)
