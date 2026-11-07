from __future__ import annotations

import numpy as np
import pytest

from ci_testbed.errors import ConfigurationError, TargetNotFound
from ci_testbed.geometry import Pixel
from ci_testbed.perception.blob_detector import BlobDetector

pytestmark = pytest.mark.unit


def make_disk(
    center_u: float,
    center_v: float,
    *,
    radius: int = 5,
    intensity: float = 1.0,
    width: int = 80,
    height: int = 60,
) -> np.ndarray:
    u_coords, v_coords = np.meshgrid(np.arange(width), np.arange(height))
    image = np.zeros((height, width), dtype=np.float64)
    mask = (u_coords - center_u) ** 2 + (v_coords - center_v) ** 2 <= radius**2
    image[mask] = intensity
    return image


def test_detects_disk_centroid() -> None:
    image = make_disk(40.0, 30.0)

    pixel = BlobDetector().detect(image)

    assert pixel == Pixel(40.0, 30.0)


def test_threshold_ignores_dim_pixels() -> None:
    image = make_disk(40.0, 30.0, intensity=0.2)

    with pytest.raises(TargetNotFound):
        BlobDetector(threshold=0.5).detect(image)
    pixel = BlobDetector(threshold=0.1).detect(image)
    assert pixel.u == pytest.approx(40.0)
    assert pixel.v == pytest.approx(30.0)


def test_min_pixels_is_enforced() -> None:
    image = np.zeros((10, 10), dtype=np.float64)
    image[5, 5] = 1.0

    with pytest.raises(TargetNotFound):
        BlobDetector(min_pixels=3).detect(image)
    pixel = BlobDetector(min_pixels=1).detect(image)
    assert pixel.u == pytest.approx(5.0)
    assert pixel.v == pytest.approx(5.0)


def test_empty_image_raises() -> None:
    with pytest.raises(TargetNotFound):
        BlobDetector().detect(np.zeros((10, 10), dtype=np.float64))


def test_non_2d_image_raises() -> None:
    with pytest.raises(ValueError):
        BlobDetector().detect(np.zeros((10, 10, 3), dtype=np.float64))


@pytest.mark.parametrize(
    "options",
    [
        {"threshold": -0.1},
        {"threshold": 1.1},
        {"min_pixels": 0},
    ],
)
def test_invalid_detector_config(options: dict[str, float]) -> None:
    with pytest.raises(ConfigurationError):
        BlobDetector(**options)  # type: ignore[arg-type]
