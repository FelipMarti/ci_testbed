from __future__ import annotations

import json
from urllib.request import Request, urlopen

import pytest

from ci_testbed.adapters import HttpCamera, HttpRangeSensor, HttpRobotArm
from ci_testbed.config import CellConfig
from ci_testbed.errors import CameraFault, JointLimitError, RobotFault, SensorDropout
from ci_testbed.geometry import Vec2
from ci_testbed.perception.blob_detector import BlobDetector

pytestmark = pytest.mark.docker


def _get_json(url: str) -> object:
    with urlopen(url, timeout=5.0) as response:
        return json.loads(response.read())


def _post(url: str) -> None:
    with urlopen(Request(url, method="POST"), timeout=5.0):
        pass


def test_all_services_report_health(docker_services: dict[str, str]) -> None:
    assert _get_json(f"{docker_services['camera']}/health") == {
        "status": "ok",
        "role": "camera",
    }
    assert _get_json(f"{docker_services['robot']}/health") == {
        "status": "ok",
        "role": "robot",
    }
    assert _get_json(f"{docker_services['lidar']}/health") == {
        "status": "ok",
        "role": "lidar",
    }


def test_camera_detects_configured_target(
    http_camera: HttpCamera,
    cell_config: CellConfig,
) -> None:
    frame = http_camera.capture()
    pixel = BlobDetector().detect(frame.image)
    world = http_camera.calibration.pixel_to_world(pixel)

    assert frame.image.shape == (cell_config.camera.height, cell_config.camera.width)
    assert world.distance_to(cell_config.target_world) <= (http_camera.calibration.meters_per_pixel)


def test_camera_faults_recover_after_reset(
    http_camera: HttpCamera,
    docker_services: dict[str, str],
) -> None:
    http_camera.fail_next(1)

    with pytest.raises(CameraFault):
        http_camera.capture()

    _post(f"{docker_services['camera']}/reset")

    assert http_camera.capture().frame_id == 0


def test_lidar_distance_and_dropout(http_range_sensor: HttpRangeSensor) -> None:
    http_range_sensor.set_distance(0.5)
    assert http_range_sensor.read_distance() == pytest.approx(0.5, abs=0.01)

    http_range_sensor.drop_next(1)
    with pytest.raises(SensorDropout):
        http_range_sensor.read_distance()

    assert http_range_sensor.read_distance() == pytest.approx(0.5, abs=0.01)


def test_robot_moves_and_controls_gripper(
    http_arm: HttpRobotArm,
    cell_config: CellConfig,
) -> None:
    target = Vec2(0.3, 0.2)

    http_arm.move_to_pose(target)
    http_arm.set_gripper(True)

    assert http_arm.get_tcp_pose().position.distance_to(target) == pytest.approx(0.0, abs=1e-9)
    assert http_arm.gripper_closed is True

    http_arm.home()
    assert http_arm.get_joint_positions() == pytest.approx(cell_config.arm.home_joints)


def test_robot_rejects_invalid_motion(http_arm: HttpRobotArm) -> None:
    with pytest.raises(JointLimitError):
        http_arm.move_joints((10.0, 0.0))


def test_robot_faults_recover_after_reset(
    http_arm: HttpRobotArm,
    docker_services: dict[str, str],
) -> None:
    http_arm.fail_next(1)

    with pytest.raises(RobotFault):
        http_arm.home()

    _post(f"{docker_services['robot']}/reset")

    http_arm.home()
