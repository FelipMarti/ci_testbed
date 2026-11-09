from __future__ import annotations

import json
from dataclasses import replace

import pytest

from ci_testbed.cli import main
from ci_testbed.clock import FakeClock
from ci_testbed.config import CellConfig
from ci_testbed.control.pick_place import ControllerState, PickPlaceController
from ci_testbed.geometry import Vec2
from ci_testbed.perception.blob_detector import BlobDetector
from ci_testbed.robot.sim_arm import SimulatedArm
from ci_testbed.sensors.synthetic_camera import SyntheticCamera
from ci_testbed.sensors.synthetic_range import SyntheticRangeSensor

pytestmark = pytest.mark.system


def build_cell(
    config: CellConfig, clock: FakeClock
) -> tuple[SyntheticCamera, SyntheticRangeSensor, SimulatedArm, PickPlaceController]:
    camera = SyntheticCamera(config.camera, clock, target_world=config.target_world)
    range_sensor = SyntheticRangeSensor(config.range_sensor, clock)
    arm = SimulatedArm(config.arm, clock)
    controller = PickPlaceController(
        camera=camera,
        detector=BlobDetector(),
        calibration=camera.calibration,
        range_sensor=range_sensor,
        arm=arm,
        config=config.controller,
        clock=clock,
    )
    return camera, range_sensor, arm, controller


def test_full_cell_with_noise_and_transient_camera_fault() -> None:
    config = CellConfig(
        camera=replace(CellConfig().camera, noise_std=0.01, seed=11),
        target_world=Vec2(0.31, 0.27),
    )
    clock = FakeClock()
    camera, _, arm, controller = build_cell(config, clock)
    camera.fail_next(1)

    result = controller.run()

    assert result.success
    assert result.state is ControllerState.DONE
    assert result.target_world is not None
    assert result.target_world.distance_to(config.target_world) <= (
        2 * config.camera.meters_per_pixel
    )
    assert result.detection_attempts == 2
    assert arm.gripper_closed
    assert result.final_pose.position.distance_to(
        arm.forward_kinematics(arm.config.home_joints).position
    ) == pytest.approx(0.0, abs=1e-9)


def test_cli_writes_result_file(tmp_path, capsys) -> None:
    output = tmp_path / "nested" / "result.json"

    exit_code = main(
        [
            "--target-x",
            "0.34",
            "--target-y",
            "0.28",
            "--noise-std",
            "0.005",
            "--output",
            str(output),
        ]
    )

    assert exit_code == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["success"] is True
    assert payload["state"] == "done"
    assert "detection_attempts" in payload
    assert json.loads(capsys.readouterr().out)["success"] is True


def test_cli_returns_non_zero_on_failure(tmp_path, capsys) -> None:
    output = tmp_path / "failure.json"

    exit_code = main(
        [
            "--target-x",
            "0.34",
            "--target-y",
            "0.28",
            "--camera-faults",
            "10",
            "--output",
            str(output),
        ]
    )

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["success"] is False
    assert payload["state"] == "failed"
    assert capsys.readouterr().out
