"""A deterministic two-link planar arm with analytic kinematics."""

from __future__ import annotations

import math

from ci_testbed.clock import Clock
from ci_testbed.config import ArmConfig
from ci_testbed.errors import JointLimitError, UnreachableTargetError
from ci_testbed.geometry import Pose2D, Vec2, normalize_angle
from ci_testbed.robot.interface import JointPositions
from ci_testbed.robot.trajectory import joint_duration

_EPSILON = 1e-9


class SimulatedArm:
    """Two-link arm with joint limits, FK, IK, and deterministic motion time."""

    def __init__(self, config: ArmConfig, clock: Clock) -> None:
        self._config = config
        self._clock = clock
        self._joints: JointPositions = config.home_joints
        self._moving = False
        self._gripper_closed = False
        self._move_count = 0
        self._stop_count = 0

    @property
    def config(self) -> ArmConfig:
        return self._config

    @property
    def is_moving(self) -> bool:
        return self._moving

    @property
    def gripper_closed(self) -> bool:
        return self._gripper_closed

    @property
    def move_count(self) -> int:
        return self._move_count

    @property
    def stop_count(self) -> int:
        return self._stop_count

    def get_joint_positions(self) -> JointPositions:
        return self._joints

    def get_tcp_pose(self) -> Pose2D:
        return self.forward_kinematics(self._joints)

    def forward_kinematics(self, joints: JointPositions) -> Pose2D:
        shoulder, elbow = joints
        x = self._config.link1 * math.cos(shoulder) + self._config.link2 * math.cos(
            shoulder + elbow
        )
        y = self._config.link1 * math.sin(shoulder) + self._config.link2 * math.sin(
            shoulder + elbow
        )
        return Pose2D(position=Vec2(x, y), theta=normalize_angle(shoulder + elbow))

    def inverse_kinematics(self, target: Vec2, *, elbow_up: bool | None = None) -> JointPositions:
        distance = target.norm()
        if distance > self._config.reach + _EPSILON:
            raise UnreachableTargetError(
                f"target {target} is beyond reach {self._config.reach:.3f} m"
            )
        if distance < abs(self._config.link1 - self._config.link2) - _EPSILON:
            raise UnreachableTargetError(f"target {target} is inside the inner workspace")

        cosine = (distance**2 - self._config.link1**2 - self._config.link2**2) / (
            2.0 * self._config.link1 * self._config.link2
        )
        cosine = max(-1.0, min(1.0, cosine))
        elbow_angle = math.acos(cosine)

        candidates: list[JointPositions] = []
        for sign in (1.0, -1.0):
            elbow = sign * elbow_angle
            shoulder = math.atan2(target.y, target.x) - math.atan2(
                self._config.link2 * math.sin(elbow),
                self._config.link1 + self._config.link2 * math.cos(elbow),
            )
            candidates.append((normalize_angle(shoulder), normalize_angle(elbow)))

        valid = [candidate for candidate in candidates if self._within_limits(candidate)]
        if elbow_up is not None:
            valid = [
                candidate
                for candidate in valid
                if (candidate[1] >= 0) == elbow_up or abs(candidate[1]) < _EPSILON
            ]
        if not valid:
            raise UnreachableTargetError(f"target {target} has no valid IK solution")

        return min(valid, key=self._distance_from_current)

    def move_joints(self, target: JointPositions, speed: float | None = None) -> None:
        self._validate_joints(target)
        effective_speed = self._config.max_joint_speed if speed is None else speed
        if effective_speed <= 0:
            raise ValueError("speed must be positive")

        duration = joint_duration(self._joints, target, effective_speed)
        self._moving = True
        try:
            self._clock.sleep(duration)
            self._joints = (normalize_angle(target[0]), normalize_angle(target[1]))
            self._move_count += 1
        finally:
            self._moving = False

    def move_to_pose(self, target: Vec2, speed: float | None = None) -> None:
        joints = self.inverse_kinematics(target)
        self.move_joints(joints, speed)

    def home(self) -> None:
        self.move_joints(self._config.home_joints)

    def stop(self) -> None:
        self._stop_count += 1
        self._moving = False

    def set_gripper(self, closed: bool) -> None:
        self._gripper_closed = closed

    def _distance_from_current(self, candidate: JointPositions) -> float:
        return (candidate[0] - self._joints[0]) ** 2 + (candidate[1] - self._joints[1]) ** 2

    def _within_limits(self, joints: JointPositions) -> bool:
        return all(
            lower - _EPSILON <= value <= upper + _EPSILON
            for value, lower, upper in zip(
                joints, self._config.joint_min, self._config.joint_max, strict=True
            )
        )

    def _validate_joints(self, joints: JointPositions) -> None:
        for index, (value, lower, upper) in enumerate(
            zip(joints, self._config.joint_min, self._config.joint_max, strict=True)
        ):
            if not math.isfinite(value):
                raise JointLimitError(f"joint {index} target is not finite")
            if not lower - _EPSILON <= value <= upper + _EPSILON:
                raise JointLimitError(
                    f"joint {index} target {value:.3f} is outside [{lower:.3f}, {upper:.3f}]"
                )
