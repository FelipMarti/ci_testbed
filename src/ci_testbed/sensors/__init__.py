"""Sensor interfaces and deterministic simulated implementations."""

from ci_testbed.sensors.interface import Camera, FloatArray, Frame, RangeSensor
from ci_testbed.sensors.synthetic_camera import SyntheticCamera
from ci_testbed.sensors.synthetic_range import SyntheticRangeSensor

__all__ = [
    "Camera",
    "FloatArray",
    "Frame",
    "RangeSensor",
    "SyntheticCamera",
    "SyntheticRangeSensor",
]
