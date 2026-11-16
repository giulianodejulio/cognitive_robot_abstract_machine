"""
Known PAL no-return values certify range without disguising invalid measurements.
"""

# %% imports
import numpy as np
import pytest

from experiments.robocat_snm.geometry import SegmentVisibility
from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.spatial_types import Point3

from .test_geometry import clear_scan


# %% source-specific no-return semantics
def test_confirmed_no_return_clears_a_section_within_sensor_range(
    clear_scan: LidarReading,
) -> None:
    """
    An in-range empty scan differs from a scan containing invalid returns.
    """
    clear_scan.ranges[:] = np.inf
    start = Point3(1, -0.3, 0, reference_frame=clear_scan.reference_frame)
    end = Point3(1, 0.3, 0, reference_frame=clear_scan.reference_frame)
    assert (
        SegmentVisibility(no_return_range=25.0).evaluate(clear_scan, start, end) is True
    )


@pytest.mark.parametrize("value", [np.nan, 0.0, 11.0])
def test_invalid_or_masked_rays_remain_unknown(
    clear_scan: LidarReading, value: float
) -> None:
    """
    Enabling no-return semantics never turns invalid or masked readings into free space.
    """
    clear_scan.ranges[:] = np.inf
    clear_scan.ranges[len(clear_scan.ranges) // 2] = value
    start = Point3(1, -0.3, 0, reference_frame=clear_scan.reference_frame)
    end = Point3(1, 0.3, 0, reference_frame=clear_scan.reference_frame)
    assert (
        SegmentVisibility(no_return_range=25.0).evaluate(clear_scan, start, end) is None
    )


def test_too_close_or_measured_obstacle_blocks_even_with_no_returns(
    clear_scan: LidarReading,
) -> None:
    """
    A too-close signal cannot be overruled by otherwise empty beams.
    """
    clear_scan.ranges[:] = np.inf
    clear_scan.ranges[len(clear_scan.ranges) // 2] = -np.inf
    start = Point3(1, -0.3, 0, reference_frame=clear_scan.reference_frame)
    end = Point3(1, 0.3, 0, reference_frame=clear_scan.reference_frame)
    assert (
        SegmentVisibility(no_return_range=25.0).evaluate(clear_scan, start, end)
        is False
    )


def test_no_return_does_not_certify_beyond_its_configured_range(
    clear_scan: LidarReading,
) -> None:
    """
    A ray with no hit only certifies the range actually searched by the sensor/filter.
    """
    clear_scan.ranges[:] = np.inf
    start = Point3(1, -0.3, 0, reference_frame=clear_scan.reference_frame)
    end = Point3(1, 0.3, 0, reference_frame=clear_scan.reference_frame)
    assert (
        SegmentVisibility(no_return_range=0.8).evaluate(clear_scan, start, end) is None
    )


def test_finite_obstacle_still_blocks_among_no_returns(
    clear_scan: LidarReading,
) -> None:
    """
    A door or person detected by one valid beam blocks the otherwise empty section.
    """
    clear_scan.ranges[:] = np.inf
    clear_scan.ranges[len(clear_scan.ranges) // 2] = 0.5
    start = Point3(1, -0.3, 0, reference_frame=clear_scan.reference_frame)
    end = Point3(1, 0.3, 0, reference_frame=clear_scan.reference_frame)
    assert (
        SegmentVisibility(no_return_range=25.0).evaluate(clear_scan, start, end)
        is False
    )
