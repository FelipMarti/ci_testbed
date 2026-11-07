from __future__ import annotations

import pytest

from ci_testbed.clock import FakeClock
from ci_testbed.config import CellConfig
from ci_testbed.control.pick_place import PickPlaceController
from ci_testbed.geometry import CameraCalibration
from ci_testbed.perception.blob_detector import BlobDetector
from ci_testbed.robot.sim_arm import SimulatedArm
from ci_testbed.sensors.synthetic_camera import SyntheticCamera
from ci_testbed.sensors.synthetic_range import SyntheticRangeSensor


@pytest.fixture
def fake_clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def cell_config() -> CellConfig:
    return CellConfig()


@pytest.fixture
def calibration(cell_config: CellConfig) -> CameraCalibration:
    return cell_config.camera.calibration()


@pytest.fixture
def camera(
    cell_config: CellConfig,
    fake_clock: FakeClock,
    calibration: CameraCalibration,
) -> SyntheticCamera:
    return SyntheticCamera(
        cell_config.camera,
        fake_clock,
        calibration=calibration,
        target_world=cell_config.target_world,
    )


@pytest.fixture
def range_sensor(cell_config: CellConfig, fake_clock: FakeClock) -> SyntheticRangeSensor:
    return SyntheticRangeSensor(cell_config.range_sensor, fake_clock)


@pytest.fixture
def arm(cell_config: CellConfig, fake_clock: FakeClock) -> SimulatedArm:
    return SimulatedArm(cell_config.arm, fake_clock)


@pytest.fixture
def detector() -> BlobDetector:
    return BlobDetector()


@pytest.fixture
def controller(
    camera: SyntheticCamera,
    detector: BlobDetector,
    calibration: CameraCalibration,
    range_sensor: SyntheticRangeSensor,
    arm: SimulatedArm,
    cell_config: CellConfig,
    fake_clock: FakeClock,
) -> PickPlaceController:
    return PickPlaceController(
        camera=camera,
        detector=detector,
        calibration=calibration,
        range_sensor=range_sensor,
        arm=arm,
        config=cell_config.controller,
        clock=fake_clock,
    )
