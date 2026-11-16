"""
Only explicit, geometrically bounded isolated scan gaps may be accepted.
"""

# %% imports
import math

import numpy as np
import pytest

from experiments.robocat_snm.application import EntryOptions
from experiments.robocat_snm.geometry import SegmentVisibility
from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.datastructures.scan_pattern import ScanPattern
from semantic_digital_twin.spatial_types import Point3
from semantic_digital_twin.world import World


# %% recorded-beam policy
@pytest.fixture
def dense_scan() -> LidarReading:
    """
    Provide a dense planar sweep with measured returns behind the segment.
    """
    world = World.create_with_root_body("laser")
    pattern = ScanPattern(-math.pi / 2, math.pi / 2, math.pi / 1800, 0.05, 10.0)
    return LidarReading(world.root, pattern, np.full(pattern.beam_count, 3.0))


def test_opt_in_accepts_a_single_bounded_gap(dense_scan: LidarReading) -> None:
    """
    A small interior missing ray with valid clear neighbours may be tolerated.
    """
    dense_scan.ranges[len(dense_scan.ranges) // 2] = np.inf
    start = Point3(1.0, -0.3, 0.0, reference_frame=dense_scan.reference_frame)
    end = Point3(1.0, 0.3, 0.0, reference_frame=dense_scan.reference_frame)
    assert SegmentVisibility().evaluate(dense_scan, start, end) is None
    assert (
        SegmentVisibility(allow_isolated_gaps=True).evaluate(dense_scan, start, end)
        is True
    )


@pytest.mark.parametrize("failure", ["consecutive", "wide", "edge"])
def test_gaps_remain_unknown_without_bounded_neighbours(
    dense_scan: LidarReading, failure: str
) -> None:
    """
    Opt-in does not excuse clusters, large physical gaps or unseen boundaries.
    """
    start = Point3(1.0, -0.3, 0.0, reference_frame=dense_scan.reference_frame)
    end = Point3(1.0, 0.3, 0.0, reference_frame=dense_scan.reference_frame)
    indices = np.flatnonzero(np.isfinite(dense_scan.distances_to_segment(start, end)))
    centre = len(dense_scan.ranges) // 2
    dense_scan.ranges[centre] = np.inf
    if failure == "consecutive":
        dense_scan.ranges[centre + 1] = np.inf
    if failure == "edge":
        dense_scan.ranges[indices[0]] = np.inf
    evaluator = SegmentVisibility(
        allow_isolated_gaps=True, maximum_gap_width=0.001 if failure == "wide" else 0.01
    )
    assert evaluator.evaluate(dense_scan, start, end) is None


def test_a_measured_obstacle_is_not_hidden_by_a_missing_ray(
    dense_scan: LidarReading,
) -> None:
    """
    A finite obstacle vetoes the corridor even if another ray is invalid.
    """
    centre = len(dense_scan.ranges) // 2
    dense_scan.ranges[centre] = np.inf
    dense_scan.ranges[centre + 5] = 0.5
    start = Point3(1.0, -0.3, 0.0, reference_frame=dense_scan.reference_frame)
    end = Point3(1.0, 0.3, 0.0, reference_frame=dense_scan.reference_frame)
    assert (
        SegmentVisibility(allow_isolated_gaps=True).evaluate(dense_scan, start, end)
        is False
    )


def test_isolated_gap_mode_requires_explicit_cli_opt_in() -> None:
    """
    Normal entry retains strict handling unless the operator supplies the flag.
    """
    assert EntryOptions.parse([]).allow_isolated_scan_gaps is False
    assert (
        EntryOptions.parse(["--allow-isolated-scan-gaps"]).allow_isolated_scan_gaps
        is True
    )
