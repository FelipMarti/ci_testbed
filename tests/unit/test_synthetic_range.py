from __future__ import annotations

import pytest

from ci_testbed.clock import FakeClock
from ci_testbed.config import RangeSensorConfig
from ci_testbed.errors import SensorDropout
from ci_testbed.sensors.synthetic_range import SyntheticRangeSensor

pytestmark = pytest.mark.unit


def test_default_reading_is_max_range() -> None:
    sensor = SyntheticRangeSensor(RangeSensorConfig(noise_std=0.0, max_range=2.0), FakeClock())

    assert sensor.read_distance() == pytest.approx(2.0)
    assert sensor.last_reading == pytest.approx(2.0)


def test_configured_distance_is_returned() -> None:
    sensor = SyntheticRangeSensor(RangeSensorConfig(noise_std=0.0), FakeClock())

    sensor.set_distance(0.4)

    assert sensor.read_distance() == pytest.approx(0.4)


@pytest.mark.parametrize("distance", [-0.1, 2.1])
def test_invalid_distance_is_rejected(distance: float) -> None:
    sensor = SyntheticRangeSensor(RangeSensorConfig(), FakeClock())

    with pytest.raises(ValueError):
        sensor.set_distance(distance)


def test_dropout_is_injected_once() -> None:
    sensor = SyntheticRangeSensor(RangeSensorConfig(noise_std=0.0), FakeClock())
    sensor.drop_next(1)

    with pytest.raises(SensorDropout):
        sensor.read_distance()

    assert sensor.read_distance() == pytest.approx(2.0)


def test_dropout_count_cannot_be_negative() -> None:
    sensor = SyntheticRangeSensor(RangeSensorConfig(), FakeClock())

    with pytest.raises(ValueError):
        sensor.drop_next(-1)


def test_noise_is_deterministic() -> None:
    config = RangeSensorConfig(noise_std=0.01, seed=3)
    first = SyntheticRangeSensor(config, FakeClock())
    second = SyntheticRangeSensor(config, FakeClock())

    assert [first.read_distance() for _ in range(5)] == [second.read_distance() for _ in range(5)]


def test_timestamp_uses_clock() -> None:
    clock = FakeClock()
    sensor = SyntheticRangeSensor(RangeSensorConfig(), clock)

    clock.advance(1.5)

    assert sensor.timestamp == pytest.approx(1.5)
