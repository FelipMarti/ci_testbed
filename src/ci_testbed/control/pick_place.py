"""A small state machine that coordinates perception, safety, and motion."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from ci_testbed.clock import Clock
from ci_testbed.config import ControllerConfig
from ci_testbed.errors import (
    CameraFault,
    RobotFault,
    SafetyAbort,
    SensorFault,
    TargetNotFound,
    TestbedError,
)
from ci_testbed.geometry import CameraCalibration, Pose2D, Vec2
from ci_testbed.perception.blob_detector import BlobDetector
from ci_testbed.robot.interface import RobotArm
from ci_testbed.sensors.interface import Camera, RangeSensor


class ControllerState(str, Enum):
    IDLE = "idle"
    DETECTING = "detecting"
    MOVING_TO_TARGET = "moving_to_target"
    GRASPING = "grasping"
    RETREATING = "retreating"
    DONE = "done"
    FAILED = "failed"
    ABORTED = "aborted"


@dataclass(frozen=True)
class PickPlaceResult:
    success: bool
    state: ControllerState
    target_world: Vec2 | None
    final_pose: Pose2D
    detection_attempts: int
    abort_reason: str | None
    duration: float

    def as_dict(self) -> dict[str, object]:
        return {
            "success": self.success,
            "state": self.state.value,
            "target_world": None
            if self.target_world is None
            else {"x": self.target_world.x, "y": self.target_world.y},
            "final_pose": {
                "x": self.final_pose.position.x,
                "y": self.final_pose.position.y,
                "theta": self.final_pose.theta,
            },
            "detection_attempts": self.detection_attempts,
            "abort_reason": self.abort_reason,
            "duration": self.duration,
        }


class PickPlaceController:
    """Detects a target, moves the arm, grasps it, and retreats home."""

    def __init__(
        self,
        *,
        camera: Camera,
        detector: BlobDetector,
        calibration: CameraCalibration,
        range_sensor: RangeSensor,
        arm: RobotArm,
        config: ControllerConfig,
        clock: Clock,
    ) -> None:
        self._camera = camera
        self._detector = detector
        self._calibration = calibration
        self._range_sensor = range_sensor
        self._arm = arm
        self._config = config
        self._clock = clock
        self._detection_attempts = 0

    def run(self) -> PickPlaceResult:
        started_at = self._clock.now()
        state = ControllerState.IDLE
        target: Vec2 | None = None
        self._detection_attempts = 0
        abort_reason: str | None = None
        success = False

        try:
            state = ControllerState.DETECTING
            self._check_safety()
            target = self._detect_target()

            state = ControllerState.MOVING_TO_TARGET
            self._check_safety()
            self._arm.move_to_pose(target)
            pose_error = self._arm.get_tcp_pose().position.distance_to(target)
            if pose_error > self._config.position_tolerance:
                raise RobotFault(
                    f"pose error {pose_error:.3f} m exceeds tolerance "
                    f"{self._config.position_tolerance:.3f} m"
                )

            state = ControllerState.GRASPING
            self._arm.set_gripper(closed=True)

            state = ControllerState.RETREATING
            self._arm.home()

            state = ControllerState.DONE
            success = True
        except SafetyAbort as exc:
            state = ControllerState.ABORTED
            abort_reason = str(exc)
            self._arm.stop()
        except TestbedError as exc:
            state = ControllerState.FAILED
            abort_reason = str(exc)
            self._arm.stop()

        return PickPlaceResult(
            success=success,
            state=state,
            target_world=target,
            final_pose=self._arm.get_tcp_pose(),
            detection_attempts=self._detection_attempts,
            abort_reason=abort_reason,
            duration=self._clock.now() - started_at,
        )

    def _detect_target(self) -> Vec2:
        last_error: Exception | None = None
        max_attempts = self._config.detection_retries + 1

        for attempt in range(1, max_attempts + 1):
            self._detection_attempts = attempt
            try:
                frame = self._camera.capture()
                pixel = self._detector.detect(frame.image)
                return self._calibration.pixel_to_world(pixel)
            except (CameraFault, TargetNotFound) as exc:
                last_error = exc
                if attempt < max_attempts:
                    self._clock.sleep(self._config.retry_backoff)

        if last_error is None:
            raise TargetNotFound("target detection failed without a specific error")
        raise last_error

    def _check_safety(self) -> None:
        try:
            distance = self._range_sensor.read_distance()
        except SensorFault as exc:
            raise SafetyAbort(f"safety sensor unavailable: {exc}") from exc
        if distance < self._config.safety_clearance:
            raise SafetyAbort(
                f"obstacle at {distance:.3f} m is inside safety clearance "
                f"{self._config.safety_clearance:.3f} m"
            )
