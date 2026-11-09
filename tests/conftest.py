from __future__ import annotations

import os
import subprocess
from collections.abc import Iterator
from pathlib import Path
from urllib.request import Request, urlopen

import pytest

from ci_testbed.adapters import HttpCamera, HttpRangeSensor, HttpRobotArm
from ci_testbed.clock import FakeClock
from ci_testbed.config import CellConfig
from ci_testbed.control.pick_place import PickPlaceController
from ci_testbed.geometry import CameraCalibration
from ci_testbed.perception.blob_detector import BlobDetector
from ci_testbed.robot.sim_arm import SimulatedArm
from ci_testbed.sensors.synthetic_camera import SyntheticCamera
from ci_testbed.sensors.synthetic_range import SyntheticRangeSensor

ROOT = Path(__file__).resolve().parents[1]
COMPOSE_FILE = ROOT / "docker" / "docker-compose.yml"


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


def _compose(
    *args: str,
    env: dict[str, str],
    check: bool = True,
    stdout: object | None = None,
) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(
        ["docker", "compose", "-f", str(COMPOSE_FILE), *args],
        check=check,
        env=env,
        stdout=stdout,
        stderr=subprocess.STDOUT if stdout is not None else None,
    )


def _post(base_url: str, path: str) -> None:
    request = Request(f"{base_url}{path}", method="POST")
    with urlopen(request, timeout=5.0):
        pass


@pytest.fixture(scope="session")
def docker_services() -> Iterator[dict[str, str]]:
    env = os.environ.copy()
    env.setdefault("CI_TESTBED_MOCKS_IMAGE", "ci-testbed-mocks:local")
    reports = ROOT / "reports"
    reports.mkdir(exist_ok=True)

    try:
        up_args = ["up", "-d", "--wait"]
        if env.get("CI_TESTBED_SKIP_BUILD") != "1":
            _compose("build", env=env)
        else:
            up_args.append("--no-build")
        _compose(*up_args, env=env)
        yield {
            "camera": "http://127.0.0.1:8001",
            "robot": "http://127.0.0.1:8002",
            "lidar": "http://127.0.0.1:8003",
        }
    finally:
        with (reports / "docker-compose.log").open("w", encoding="utf-8") as log_file:
            _compose("logs", "--no-color", env=env, check=False, stdout=log_file)
        _compose("down", "-v", env=env, check=False)


@pytest.fixture
def reset_mocks(docker_services: dict[str, str]) -> Iterator[None]:
    for base_url in docker_services.values():
        _post(base_url, "/reset")
    yield
    for base_url in docker_services.values():
        _post(base_url, "/reset")


@pytest.fixture
def http_camera(
    docker_services: dict[str, str],
    reset_mocks: None,
    cell_config: CellConfig,
    calibration: CameraCalibration,
) -> HttpCamera:
    return HttpCamera(
        docker_services["camera"],
        calibration=calibration,
        target_world=cell_config.target_world,
    )


@pytest.fixture
def http_range_sensor(docker_services: dict[str, str], reset_mocks: None) -> HttpRangeSensor:
    return HttpRangeSensor(docker_services["lidar"])


@pytest.fixture
def http_arm(docker_services: dict[str, str], reset_mocks: None) -> HttpRobotArm:
    return HttpRobotArm(docker_services["robot"])
