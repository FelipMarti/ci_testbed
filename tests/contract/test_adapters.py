from __future__ import annotations

import numpy as np
import pytest

from ci_testbed.adapters import HttpCamera, HttpRangeSensor, HttpRobotArm
from ci_testbed.clock import FakeClock
from ci_testbed.config import ArmConfig, CameraConfig, CellConfig
from ci_testbed.errors import CameraFault, JointLimitError, SensorDropout
from ci_testbed.geometry import CameraCalibration, Vec2
from ci_testbed.perception.blob_detector import BlobDetector
from ci_testbed.robot.sim_arm import SimulatedArm
from ci_testbed.sensors.synthetic_camera import SyntheticCamera
from ci_testbed.sensors.synthetic_range import SyntheticRangeSensor

pytestmark = pytest.mark.contract

DOCKER = pytest.mark.docker


@pytest.fixture(params=["synthetic", pytest.param("http", marks=DOCKER)])
def camera_under_test(
    request: pytest.FixtureRequest,
    cell_config: CellConfig,
    fake_clock: FakeClock,
    calibration: CameraCalibration,
) -> SyntheticCamera | HttpCamera:
    if request.param == "synthetic":
        return SyntheticCamera(
            cell_config.camera,
            fake_clock,
            calibration=calibration,
            target_world=cell_config.target_world,
        )
    if request.param == "http":
        request.getfixturevalue("reset_mocks")
        urls = request.getfixturevalue("docker_services")
        return HttpCamera(
            urls["camera"],
            calibration=calibration,
            target_world=cell_config.target_world,
        )
    raise AssertionError(f"unknown camera implementation: {request.param}")


@pytest.fixture(params=["simulated", pytest.param("http", marks=DOCKER)])
def arm_under_test(
    request: pytest.FixtureRequest,
    cell_config: CellConfig,
    fake_clock: FakeClock,
) -> SimulatedArm | HttpRobotArm:
    if request.param == "simulated":
        return SimulatedArm(cell_config.arm, fake_clock)
    if request.param == "http":
        request.getfixturevalue("reset_mocks")
        urls = request.getfixturevalue("docker_services")
        return HttpRobotArm(urls["robot"])
    raise AssertionError(f"unknown arm implementation: {request.param}")


@pytest.fixture(params=["synthetic", pytest.param("http", marks=DOCKER)])
def range_sensor_under_test(
    request: pytest.FixtureRequest,
    cell_config: CellConfig,
    fake_clock: FakeClock,
) -> SyntheticRangeSensor | HttpRangeSensor:
    if request.param == "synthetic":
        return SyntheticRangeSensor(cell_config.range_sensor, fake_clock)
    if request.param == "http":
        request.getfixturevalue("reset_mocks")
        urls = request.getfixturevalue("docker_services")
        return HttpRangeSensor(urls["lidar"])
    raise AssertionError(f"unknown range sensor implementation: {request.param}")


def test_every_camera_returns_valid_frames(
    camera_under_test: SyntheticCamera | HttpCamera,
) -> None:
    frame = camera_under_test.capture()

    assert isinstance(frame.image, np.ndarray)
    assert frame.image.ndim == 2
    assert frame.timestamp >= 0.0
    assert frame.frame_id >= 0


def test_every_camera_has_monotonic_frame_ids(
    camera_under_test: SyntheticCamera | HttpCamera,
) -> None:
    first = camera_under_test.capture()
    second = camera_under_test.capture()

    assert second.frame_id == first.frame_id + 1
    assert second.timestamp >= first.timestamp


def test_every_camera_sees_the_configured_target(
    camera_under_test: SyntheticCamera | HttpCamera,
) -> None:
    frame = camera_under_test.capture()

    pixel = BlobDetector().detect(frame.image)
    world = camera_under_test.calibration.pixel_to_world(pixel)

    assert world.distance_to(camera_under_test.target_world) <= (
        camera_under_test.calibration.meters_per_pixel
    )


def test_every_camera_recovers_after_a_fault(
    camera_under_test: SyntheticCamera | HttpCamera,
) -> None:
    camera_under_test.fail_next(1)

    with pytest.raises(CameraFault):
        camera_under_test.capture()

    assert camera_under_test.capture().frame_id == 0


def test_every_arm_can_home_and_report_pose(
    arm_under_test: SimulatedArm | HttpRobotArm,
    cell_config: CellConfig,
) -> None:
    arm_under_test.home()

    assert arm_under_test.get_joint_positions() == pytest.approx(cell_config.arm.home_joints)
    assert arm_under_test.get_tcp_pose().position.norm() > 0.0


def test_every_arm_reaches_valid_cartesian_targets(
    arm_under_test: SimulatedArm | HttpRobotArm,
) -> None:
    target = Vec2(0.3, 0.2)

    arm_under_test.move_to_pose(target)

    assert arm_under_test.get_tcp_pose().position.distance_to(target) == pytest.approx(
        0.0, abs=1e-9
    )


def test_every_camera_can_change_target(
    camera_under_test: SyntheticCamera | HttpCamera,
) -> None:
    target = Vec2(0.30, 0.25)

    camera_under_test.set_target_world(target)
    frame = camera_under_test.capture()
    pixel = BlobDetector().detect(frame.image)
    world = camera_under_test.calibration.pixel_to_world(pixel)

    assert world.distance_to(target) <= camera_under_test.calibration.meters_per_pixel


def test_every_arm_rejects_joint_limit_violations(
    arm_under_test: SimulatedArm | HttpRobotArm,
) -> None:
    with pytest.raises(JointLimitError):
        arm_under_test.move_joints((10.0, 0.0))


def test_every_range_sensor_returns_valid_distance(
    range_sensor_under_test: SyntheticRangeSensor | HttpRangeSensor,
    cell_config: CellConfig,
) -> None:
    distance = range_sensor_under_test.read_distance()

    assert 0.0 <= distance <= cell_config.range_sensor.max_range


def test_every_range_sensor_can_set_distance(
    range_sensor_under_test: SyntheticRangeSensor | HttpRangeSensor,
) -> None:
    target = 0.75

    range_sensor_under_test.set_distance(target)

    assert range_sensor_under_test.read_distance() == pytest.approx(target, abs=0.01)


def test_every_range_sensor_recovers_after_a_dropout(
    range_sensor_under_test: SyntheticRangeSensor | HttpRangeSensor,
) -> None:
    range_sensor_under_test.drop_next(1)

    with pytest.raises(SensorDropout):
        range_sensor_under_test.read_distance()

    assert range_sensor_under_test.read_distance() >= 0.0


def test_camera_config_is_shared_across_adapters(
    cell_config: CellConfig,
    camera_under_test: SyntheticCamera | HttpCamera,
) -> None:
    assert isinstance(CameraConfig(), type(cell_config.camera))
    assert isinstance(ArmConfig(), type(cell_config.arm))
    assert camera_under_test.calibration.width == cell_config.camera.width
