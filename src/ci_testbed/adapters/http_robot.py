"""Robot arm adapter for the containerised robot mock."""

from __future__ import annotations

from dataclasses import dataclass

from ci_testbed.adapters._http import (
    HttpClient,
    HttpRequestError,
    as_bool,
    as_float,
    as_mapping,
    as_sequence,
    required,
)
from ci_testbed.errors import JointLimitError, RobotFault, UnreachableTargetError
from ci_testbed.geometry import Pose2D, Vec2
from ci_testbed.robot.interface import JointPositions


@dataclass(frozen=True)
class RobotState:
    joints: JointPositions
    tcp: Pose2D
    is_moving: bool
    gripper_closed: bool


class HttpRobotArm:
    """Robot arm implementation backed by ``robot-mock`` over HTTP."""

    def __init__(self, base_url: str, *, timeout: float = 5.0) -> None:
        self._client = HttpClient(base_url, timeout=timeout)

    def get_joint_positions(self) -> JointPositions:
        return self._state().joints

    def get_tcp_pose(self) -> Pose2D:
        return self._state().tcp

    def move_joints(self, target: JointPositions, speed: float | None = None) -> None:
        self._call(
            "/move_joints",
            {"joints": [target[0], target[1]], "speed": speed},
            allow_unreachable=False,
        )

    def move_to_pose(self, target: Vec2, speed: float | None = None) -> None:
        self._call(
            "/move_to_pose",
            {"x": target.x, "y": target.y, "speed": speed},
            allow_unreachable=True,
        )

    def home(self) -> None:
        self._call("/home", {}, allow_unreachable=False)

    def stop(self) -> None:
        self._call("/stop", {}, allow_unreachable=False)

    def set_gripper(self, closed: bool) -> None:
        self._call("/gripper", {"closed": closed}, allow_unreachable=False)

    def fail_next(self, count: int = 1) -> None:
        if count < 0:
            raise ValueError("count must be non-negative")
        self._call("/faults", {"count": count}, allow_unreachable=False)

    @property
    def is_moving(self) -> bool:
        return self._state().is_moving

    @property
    def gripper_closed(self) -> bool:
        return self._state().gripper_closed

    def _state(self) -> RobotState:
        try:
            payload = self._client.get("/state")
            return _parse_state(payload)
        except HttpRequestError as exc:
            raise RobotFault(f"robot state request failed: {exc}") from exc

    def _call(
        self,
        path: str,
        payload: dict[str, object],
        *,
        allow_unreachable: bool,
    ) -> None:
        try:
            self._client.post(path, payload)
        except HttpRequestError as exc:
            message = str(exc)
            if exc.status == 400 and exc.code == "joint_limit":
                raise JointLimitError(message) from exc
            if allow_unreachable and exc.status == 400 and exc.code == "unreachable_target":
                raise UnreachableTargetError(message) from exc
            if exc.status == 503 and exc.code == "robot_fault":
                raise RobotFault(message) from exc
            raise RobotFault(f"robot service request failed: {exc}") from exc


def _parse_state(payload: object) -> RobotState:
    data = as_mapping(payload, "robot state")
    joints = as_sequence(required(data, "joints", "robot state"), "robot state.joints")
    if len(joints) != 2:
        raise HttpRequestError("robot state.joints: expected two values")
    tcp = as_mapping(required(data, "tcp", "robot state"), "robot state.tcp")
    return RobotState(
        joints=(
            as_float(joints[0], "robot state.joints[0]"),
            as_float(joints[1], "robot state.joints[1]"),
        ),
        tcp=Pose2D(
            position=Vec2(
                as_float(required(tcp, "x", "robot state.tcp"), "robot state.tcp.x"),
                as_float(required(tcp, "y", "robot state.tcp"), "robot state.tcp.y"),
            ),
            theta=as_float(required(tcp, "theta", "robot state.tcp"), "robot state.tcp.theta"),
        ),
        is_moving=as_bool(
            required(data, "is_moving", "robot state"),
            "robot state.is_moving",
        ),
        gripper_closed=as_bool(
            required(data, "gripper_closed", "robot state"),
            "robot state.gripper_closed",
        ),
    )
