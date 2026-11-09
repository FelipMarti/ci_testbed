from __future__ import annotations

import argparse
import time
from threading import Lock
from typing import Literal

import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field

from ci_testbed.config import CellConfig
from ci_testbed.errors import (
    CameraFault,
    JointLimitError,
    RobotFault,
    SensorDropout,
    UnreachableTargetError,
)
from ci_testbed.geometry import Pose2D, Vec2
from ci_testbed.robot.interface import JointPositions
from ci_testbed.robot.sim_arm import SimulatedArm
from ci_testbed.sensors.synthetic_camera import SyntheticCamera
from ci_testbed.sensors.synthetic_range import SyntheticRangeSensor

Role = Literal["camera", "robot", "lidar"]


class TargetRequest(BaseModel):
    x: float
    y: float


class JointsRequest(BaseModel):
    joints: tuple[float, float]
    speed: float | None = None


class PoseRequest(BaseModel):
    x: float
    y: float
    speed: float | None = None


class GripperRequest(BaseModel):
    closed: bool


class DistanceRequest(BaseModel):
    distance: float = Field(ge=0.0)


class FaultsRequest(BaseModel):
    count: int = Field(default=1, ge=0)


class InstantClock:
    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        if seconds < 0:
            raise ValueError("cannot sleep for a negative duration")


class FaultyArm:
    def __init__(self, arm: SimulatedArm) -> None:
        self._arm = arm
        self._faults_remaining = 0

    def fail_next(self, count: int = 1) -> None:
        self._faults_remaining += count

    def _check_fault(self) -> None:
        if self._faults_remaining > 0:
            self._faults_remaining -= 1
            raise RobotFault("injected robot motion fault")

    def get_joint_positions(self) -> JointPositions:
        return self._arm.get_joint_positions()

    def get_tcp_pose(self) -> Pose2D:
        return self._arm.get_tcp_pose()

    def move_joints(self, target: JointPositions, speed: float | None = None) -> None:
        self._check_fault()
        self._arm.move_joints(target, speed)

    def move_to_pose(self, target: Vec2, speed: float | None = None) -> None:
        self._check_fault()
        self._arm.move_to_pose(target, speed)

    def home(self) -> None:
        self._check_fault()
        self._arm.home()

    def stop(self) -> None:
        self._arm.stop()

    def set_gripper(self, closed: bool) -> None:
        self._arm.set_gripper(closed)

    @property
    def is_moving(self) -> bool:
        return self._arm.is_moving

    @property
    def gripper_closed(self) -> bool:
        return self._arm.gripper_closed


def _error(status_code: int, code: str, exc: Exception) -> HTTPException:
    return HTTPException(
        status_code=status_code,
        detail={"error": code, "message": str(exc)},
    )


def _pose_payload(pose: Pose2D) -> dict[str, float]:
    return {"x": pose.position.x, "y": pose.position.y, "theta": pose.theta}


def create_app(role: Role) -> FastAPI:
    app = FastAPI(title=f"ci-testbed {role} mock", version="0.1.0")
    config = CellConfig()
    clock = InstantClock()
    lock = Lock()

    camera: SyntheticCamera | None = None
    range_sensor: SyntheticRangeSensor | None = None
    arm: FaultyArm | None = None

    def reset_camera() -> SyntheticCamera:
        return SyntheticCamera(
            config.camera,
            clock,
            calibration=config.camera.calibration(),
            target_world=config.target_world,
        )

    def reset_range_sensor() -> SyntheticRangeSensor:
        return SyntheticRangeSensor(config.range_sensor, clock)

    def reset_arm() -> FaultyArm:
        return FaultyArm(SimulatedArm(config.arm, clock))

    if role == "camera":
        camera = reset_camera()
    elif role == "lidar":
        range_sensor = reset_range_sensor()
    else:
        arm = reset_arm()

    @app.get("/health")
    def health() -> dict[str, str]:
        return {"status": "ok", "role": role}

    @app.post("/reset")
    def reset() -> dict[str, str]:
        nonlocal camera, range_sensor, arm
        with lock:
            if role == "camera":
                camera = reset_camera()
            elif role == "lidar":
                range_sensor = reset_range_sensor()
            else:
                arm = reset_arm()
        return {"status": "ok", "role": role}

    if role == "camera":

        @app.get("/frame")
        def frame() -> dict[str, object]:
            assert camera is not None
            with lock:
                try:
                    captured = camera.capture()
                except CameraFault as exc:
                    raise _error(503, "camera_fault", exc) from exc
            return {
                "image": captured.image.tolist(),
                "timestamp": captured.timestamp,
                "frame_id": captured.frame_id,
            }

        @app.post("/target")
        def set_target(request: TargetRequest) -> dict[str, float]:
            assert camera is not None
            with lock:
                camera.set_target_world(Vec2(request.x, request.y))
            return {"x": request.x, "y": request.y}

        @app.get("/config")
        def camera_config() -> dict[str, object]:
            assert camera is not None
            calibration = camera.calibration
            return {
                "width": calibration.width,
                "height": calibration.height,
                "meters_per_pixel": calibration.meters_per_pixel,
                "world_origin": {
                    "x": calibration.world_origin.x,
                    "y": calibration.world_origin.y,
                },
                "target": {"x": camera.target_world.x, "y": camera.target_world.y},
            }

        @app.post("/faults")
        def inject_faults(request: FaultsRequest) -> dict[str, int]:
            assert camera is not None
            with lock:
                camera.fail_next(request.count)
            return {"count": request.count}

    if role == "lidar":

        @app.get("/distance")
        def distance() -> dict[str, float]:
            assert range_sensor is not None
            with lock:
                try:
                    reading = range_sensor.read_distance()
                except SensorDropout as exc:
                    raise _error(503, "sensor_dropout", exc) from exc
            return {"distance": reading}

        @app.post("/distance")
        def set_distance(request: DistanceRequest) -> dict[str, float]:
            assert range_sensor is not None
            with lock:
                try:
                    range_sensor.set_distance(request.distance)
                except ValueError as exc:
                    raise _error(400, "invalid_distance", exc) from exc
            return {"distance": request.distance}

        @app.post("/faults")
        def inject_faults(request: FaultsRequest) -> dict[str, int]:
            assert range_sensor is not None
            with lock:
                range_sensor.drop_next(request.count)
            return {"count": request.count}

    if role == "robot":

        @app.get("/state")
        def state() -> dict[str, object]:
            assert arm is not None
            with lock:
                return {
                    "joints": list(arm.get_joint_positions()),
                    "tcp": _pose_payload(arm.get_tcp_pose()),
                    "is_moving": arm.is_moving,
                    "gripper_closed": arm.gripper_closed,
                }

        @app.post("/move_joints")
        def move_joints(request: JointsRequest) -> dict[str, object]:
            assert arm is not None
            with lock:
                try:
                    arm.move_joints(request.joints, request.speed)
                except JointLimitError as exc:
                    raise _error(400, "joint_limit", exc) from exc
                except RobotFault as exc:
                    raise _error(503, "robot_fault", exc) from exc
                except ValueError as exc:
                    raise _error(400, "invalid_request", exc) from exc
                return {
                    "joints": list(arm.get_joint_positions()),
                    "tcp": _pose_payload(arm.get_tcp_pose()),
                    "is_moving": arm.is_moving,
                    "gripper_closed": arm.gripper_closed,
                }

        @app.post("/move_to_pose")
        def move_to_pose(request: PoseRequest) -> dict[str, object]:
            assert arm is not None
            with lock:
                try:
                    arm.move_to_pose(Vec2(request.x, request.y), request.speed)
                except UnreachableTargetError as exc:
                    raise _error(400, "unreachable_target", exc) from exc
                except JointLimitError as exc:
                    raise _error(400, "joint_limit", exc) from exc
                except RobotFault as exc:
                    raise _error(503, "robot_fault", exc) from exc
                except ValueError as exc:
                    raise _error(400, "invalid_request", exc) from exc
                return {
                    "joints": list(arm.get_joint_positions()),
                    "tcp": _pose_payload(arm.get_tcp_pose()),
                    "is_moving": arm.is_moving,
                    "gripper_closed": arm.gripper_closed,
                }

        @app.post("/home")
        def home() -> dict[str, object]:
            assert arm is not None
            with lock:
                try:
                    arm.home()
                except RobotFault as exc:
                    raise _error(503, "robot_fault", exc) from exc
                return {
                    "joints": list(arm.get_joint_positions()),
                    "tcp": _pose_payload(arm.get_tcp_pose()),
                    "is_moving": arm.is_moving,
                    "gripper_closed": arm.gripper_closed,
                }

        @app.post("/stop")
        def stop() -> dict[str, object]:
            assert arm is not None
            with lock:
                arm.stop()
                return {
                    "joints": list(arm.get_joint_positions()),
                    "tcp": _pose_payload(arm.get_tcp_pose()),
                    "is_moving": arm.is_moving,
                    "gripper_closed": arm.gripper_closed,
                }

        @app.post("/gripper")
        def gripper(request: GripperRequest) -> dict[str, object]:
            assert arm is not None
            with lock:
                arm.set_gripper(request.closed)
                return {
                    "joints": list(arm.get_joint_positions()),
                    "tcp": _pose_payload(arm.get_tcp_pose()),
                    "is_moving": arm.is_moving,
                    "gripper_closed": arm.gripper_closed,
                }

        @app.post("/faults")
        def inject_faults(request: FaultsRequest) -> dict[str, int]:
            assert arm is not None
            with lock:
                arm.fail_next(request.count)
            return {"count": request.count}

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a ci-testbed HTTP mock service")
    parser.add_argument("role", choices=["camera", "robot", "lidar"])
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, required=True)
    args = parser.parse_args()
    uvicorn.run(create_app(args.role), host=args.host, port=args.port)


if __name__ == "__main__":
    main()
