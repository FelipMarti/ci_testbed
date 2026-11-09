"""HTTP adapters for the containerised mock services."""

from ci_testbed.adapters.http_camera import HttpCamera
from ci_testbed.adapters.http_range_sensor import HttpRangeSensor
from ci_testbed.adapters.http_robot import HttpRobotArm

__all__ = ["HttpCamera", "HttpRangeSensor", "HttpRobotArm"]
