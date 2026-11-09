from __future__ import annotations

import pytest

from ci_testbed.control.pick_place import ControllerState, PickPlaceController
from ci_testbed.robot.sim_arm import SimulatedArm

pytestmark = pytest.mark.integration


def test_controller_completes_pick_and_place(
    controller: PickPlaceController, arm: SimulatedArm, cell_config
) -> None:
    result = controller.run()

    assert result.success
    assert result.state is ControllerState.DONE
    assert result.target_world is not None
    assert result.target_world.distance_to(cell_config.target_world) <= (
        cell_config.camera.meters_per_pixel
    )
    assert result.final_pose.position.distance_to(
        arm.forward_kinematics(arm.config.home_joints).position
    ) == pytest.approx(0.0, abs=1e-9)
    assert arm.gripper_closed
    assert arm.move_count == 2
    assert arm.stop_count == 0
    assert result.detection_attempts == 1
    assert result.abort_reason is None
    assert result.duration > 0.0


def test_controller_returns_home(controller: PickPlaceController, arm: SimulatedArm) -> None:
    controller.run()

    assert arm.get_joint_positions() == pytest.approx(arm.config.home_joints)


def test_result_is_json_friendly(controller: PickPlaceController) -> None:
    payload = controller.run().as_dict()

    assert payload["success"] is True
    assert payload["state"] == "done"
    assert isinstance(payload["target_world"], dict)
    assert isinstance(payload["final_pose"], dict)
    assert payload["detection_attempts"] == 1
