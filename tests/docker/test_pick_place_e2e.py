from __future__ import annotations

import pytest

from ci_testbed.adapters import HttpCamera, HttpRangeSensor, HttpRobotArm
from ci_testbed.clock import FakeClock
from ci_testbed.config import CellConfig
from ci_testbed.control.pick_place import PickPlaceController
from ci_testbed.perception.blob_detector import BlobDetector

pytestmark = pytest.mark.docker


@pytest.mark.smoke
def test_pick_and_place_against_http_mocks(
    http_camera: HttpCamera,
    http_range_sensor: HttpRangeSensor,
    http_arm: HttpRobotArm,
    cell_config: CellConfig,
) -> None:
    controller = PickPlaceController(
        camera=http_camera,
        detector=BlobDetector(),
        calibration=http_camera.calibration,
        range_sensor=http_range_sensor,
        arm=http_arm,
        config=cell_config.controller,
        clock=FakeClock(),
    )

    result = controller.run()

    assert result.success is True
    assert result.state.value == "done"
    assert result.target_world is not None
    assert result.target_world.distance_to(cell_config.target_world) <= (
        http_camera.calibration.meters_per_pixel
    )
    assert http_arm.gripper_closed is True
    assert http_arm.get_joint_positions() == pytest.approx(cell_config.arm.home_joints)
