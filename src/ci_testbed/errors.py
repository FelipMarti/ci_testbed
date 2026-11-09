"""Domain exceptions shared by the simulator and its adapters."""


class TestbedError(Exception):
    """Base class for all project-specific failures."""


class ConfigurationError(TestbedError, ValueError):
    """Raised when a configuration value is invalid."""


class SensorFault(TestbedError):
    """Base class for sensor failures that may be transient."""


class CameraFault(SensorFault):
    """Raised when the camera cannot produce a frame."""


class SensorDropout(SensorFault):
    """Raised when a sensor has no reading for the current cycle."""


class RobotFault(TestbedError):
    """Base class for robot command failures."""


class JointLimitError(RobotFault):
    """Raised when a joint target violates configured limits."""


class UnreachableTargetError(RobotFault):
    """Raised when a Cartesian target is outside the robot workspace."""


class TargetNotFound(TestbedError):
    """Raised when perception cannot find the requested object."""


class SafetyAbort(TestbedError):
    """Raised when a safety check requires the current action to stop."""
