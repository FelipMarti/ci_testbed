from __future__ import annotations

import pytest

from ci_testbed.control.pick_place import (
    ControllerState,
    PickPlaceController,
    PickPlaceResult,
)
from ci_testbed.geometry import Vec2
from ci_testbed.sensors.synthetic_camera import SyntheticCamera
from ci_testbed.sensors.synthetic_range import SyntheticRangeSensor
from tests.fakes import FaultyArm, StuckArm

pytestmark = pytest.mark.integration


def test_transient_camera_fault_is_retried(
    controller: PickPlaceController, camera: SyntheticCamera
) -> None:
    camera.fail_next(1)

    result = controller.run()

    assert result.success
    assert result.state is ControllerState.DONE
    assert result.detection_attempts == 2


def test_persistent_camera_fault_fails_safely(
    controller: PickPlaceController,
    camera: SyntheticCamera,
    arm,
    cell_config,
) -> None:
    camera.fail_next(99)

    result = controller.run()

    assert not result.success
    assert result.state is ControllerState.FAILED
    assert result.detection_attempts == cell_config.controller.detection_retries + 1
    assert "injected camera failure" in (result.abort_reason or "")
    assert result.target_world is None
    assert arm.move_count == 0
    assert arm.stop_count == 1


def test_obstacle_causes_safety_abort(
    controller: PickPlaceController,
    range_sensor: SyntheticRangeSensor,
    arm,
) -> None:
    range_sensor.set_distance(0.05)

    result = controller.run()

    assert not result.success
    assert result.state is ControllerState.ABORTED
    assert result.detection_attempts == 0
    assert "safety clearance" in (result.abort_reason or "")
    assert arm.move_count == 0
    assert arm.stop_count == 1


def test_safety_sensor_dropout_aborts(
    controller: PickPlaceController, range_sensor: SyntheticRangeSensor
) -> None:
    range_sensor.drop_next(1)

    result = controller.run()

    assert not result.success
    assert result.state is ControllerState.ABORTED
    assert "safety sensor unavailable" in (result.abort_reason or "")


def test_robot_fault_fails_safely(
    camera: SyntheticCamera,
    detector,
    calibration,
    range_sensor: SyntheticRangeSensor,
    arm,
    cell_config,
    fake_clock,
) -> None:
    faulty_arm = FaultyArm(arm, fail_on_move=1)
    controller = PickPlaceController(
        camera=camera,
        detector=detector,
        calibration=calibration,
        range_sensor=range_sensor,
        arm=faulty_arm,
        config=cell_config.controller,
        clock=fake_clock,
    )

    result = controller.run()

    assert not result.success
    assert result.state is ControllerState.FAILED
    assert "injected robot motion fault" in (result.abort_reason or "")
    assert result.target_world is not None
    assert faulty_arm.stop_calls == 1
    assert arm.move_count == 0


def test_unreachable_target_fails_safely(
    controller: PickPlaceController, camera: SyntheticCamera, arm
) -> None:
    camera.set_target_world(Vec2(0.51, 0.43))

    result: PickPlaceResult = controller.run()

    assert not result.success
    assert result.state is ControllerState.FAILED
    assert result.target_world is not None
    assert "reach" in (result.abort_reason or "") or "IK" in (result.abort_reason or "")
    assert arm.move_count == 0
    assert arm.stop_count == 1


def test_pose_error_is_detected(
    camera: SyntheticCamera,
    detector,
    calibration,
    range_sensor: SyntheticRangeSensor,
    arm,
    cell_config,
    fake_clock,
) -> None:
    stuck_arm = StuckArm(arm)
    controller = PickPlaceController(
        camera=camera,
        detector=detector,
        calibration=calibration,
        range_sensor=range_sensor,
        arm=stuck_arm,
        config=cell_config.controller,
        clock=fake_clock,
    )

    result = controller.run()

    assert not result.success
    assert result.state is ControllerState.FAILED
    assert "pose error" in (result.abort_reason or "")
    assert stuck_arm.stop_calls == 1
