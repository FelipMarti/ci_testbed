from __future__ import annotations

import json
from collections.abc import Iterator
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Thread
from typing import ClassVar

import numpy as np
import pytest

from ci_testbed.adapters import HttpCamera, HttpRangeSensor, HttpRobotArm
from ci_testbed.config import CameraConfig
from ci_testbed.errors import (
    CameraFault,
    JointLimitError,
    RobotFault,
    SensorDropout,
    UnreachableTargetError,
)
from ci_testbed.geometry import Vec2

pytestmark = pytest.mark.integration

Routes = dict[tuple[str, str], tuple[int, object]]


class _StubHandler(BaseHTTPRequestHandler):
    routes: ClassVar[Routes] = {}

    def do_GET(self) -> None:
        self._respond()

    def do_POST(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        if length:
            self.rfile.read(length)
        self._respond()

    def _respond(self) -> None:
        status, payload = self.routes.get((self.command, self.path), (404, {}))
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: object) -> None:
        return


@pytest.fixture
def stub_server() -> Iterator[tuple[str, Routes]]:
    routes: Routes = {}

    class Handler(_StubHandler):
        pass

    Handler.routes = routes
    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}", routes
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_camera_adapter_parses_frames(stub_server: tuple[str, Routes]) -> None:
    url, routes = stub_server
    routes[("GET", "/frame")] = (
        200,
        {"image": [[0.0, 1.0], [0.5, 0.0]], "timestamp": 1.5, "frame_id": 3},
    )
    camera = HttpCamera(url, calibration=CameraConfig().calibration())

    frame = camera.capture()

    assert frame.image.shape == (2, 2)
    assert frame.image.dtype == np.float64
    assert frame.timestamp == 1.5
    assert frame.frame_id == 3
    assert camera.frames_captured == 1
    assert camera.last_frame is frame


def test_camera_adapter_maps_service_faults(stub_server: tuple[str, Routes]) -> None:
    url, routes = stub_server
    routes[("GET", "/frame")] = (
        503,
        {"detail": {"error": "camera_fault", "message": "injected"}},
    )
    camera = HttpCamera(url)

    with pytest.raises(CameraFault, match="injected"):
        camera.capture()


def test_range_adapter_parses_distance(stub_server: tuple[str, Routes]) -> None:
    url, routes = stub_server
    routes[("GET", "/distance")] = (200, {"distance": 0.42})
    sensor = HttpRangeSensor(url)

    assert sensor.read_distance() == pytest.approx(0.42)
    assert sensor.last_reading == pytest.approx(0.42)


def test_range_adapter_maps_dropouts(stub_server: tuple[str, Routes]) -> None:
    url, routes = stub_server
    routes[("GET", "/distance")] = (
        503,
        {"detail": {"error": "sensor_dropout", "message": "missing"}},
    )
    sensor = HttpRangeSensor(url)

    with pytest.raises(SensorDropout, match="missing"):
        sensor.read_distance()


def test_robot_adapter_parses_state(stub_server: tuple[str, Routes]) -> None:
    url, routes = stub_server
    routes[("GET", "/state")] = (
        200,
        {
            "joints": [0.25, -0.5],
            "tcp": {"x": 0.3, "y": 0.2, "theta": -0.25},
            "is_moving": False,
            "gripper_closed": True,
        },
    )
    arm = HttpRobotArm(url)

    assert arm.get_joint_positions() == pytest.approx((0.25, -0.5))
    assert arm.get_tcp_pose().position == Vec2(0.3, 0.2)
    assert arm.get_tcp_pose().theta == pytest.approx(-0.25)
    assert arm.is_moving is False
    assert arm.gripper_closed is True


def test_robot_adapter_maps_joint_limit_errors(stub_server: tuple[str, Routes]) -> None:
    url, routes = stub_server
    routes[("POST", "/move_joints")] = (
        400,
        {"detail": {"error": "joint_limit", "message": "outside limits"}},
    )
    arm = HttpRobotArm(url)

    with pytest.raises(JointLimitError, match="outside limits"):
        arm.move_joints((10.0, 0.0))


def test_robot_adapter_maps_unreachable_targets(stub_server: tuple[str, Routes]) -> None:
    url, routes = stub_server
    routes[("POST", "/move_to_pose")] = (
        400,
        {"detail": {"error": "unreachable_target", "message": "too far"}},
    )
    arm = HttpRobotArm(url)

    with pytest.raises(UnreachableTargetError, match="too far"):
        arm.move_to_pose(Vec2(10.0, 10.0))


def test_robot_adapter_maps_robot_faults(stub_server: tuple[str, Routes]) -> None:
    url, routes = stub_server
    routes[("POST", "/home")] = (
        503,
        {"detail": {"error": "robot_fault", "message": "injected"}},
    )
    arm = HttpRobotArm(url)

    with pytest.raises(RobotFault, match="injected"):
        arm.home()
