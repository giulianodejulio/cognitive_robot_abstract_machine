"""
Check configured cabin targets and measured lidar clearance.
"""

# %% imports
import math

import numpy as np
import pytest

from experiments.robocat_snm.geometry import EntryGeometry, SegmentVisibility
from experiments.robocat_snm.exceptions import EntryError
from semantic_digital_twin.datastructures.lidar_reading import LidarReading
from semantic_digital_twin.datastructures.scan_pattern import ScanPattern
from semantic_digital_twin.spatial_types import Point3, Pose
from semantic_digital_twin.world import World


# %% target and threshold
def test_target_uses_staging_axes_not_map_axes() -> None:
    """
    A north-facing staging pose places the target north of staging.
    """
    geometry = EntryGeometry(
        Pose.from_xyz_rpy(x=4, y=8, yaw=math.pi / 2), doorway_distance=0.7
    )
    assert float(geometry.target.x) == pytest.approx(4)
    assert float(geometry.target.y) == pytest.approx(8 + geometry.distance)
    assert float(geometry.target.yaw) == pytest.approx(math.pi / 2)


def test_rear_must_clear_threshold() -> None:
    """
    A chassis straddling the threshold cannot be reported fully inside.
    """
    geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.7)
    assert geometry.contains_robot(Pose.from_xyz_rpy(x=0.8)) is False
    assert geometry.contains_robot(geometry.target) is True


@pytest.mark.parametrize("distance", [0, float("nan"), 0.8])
def test_target_must_leave_room_for_rear_clearance(distance: float) -> None:
    """
    Entry goals must fit the complete chassis beyond the doorway.
    """
    with pytest.raises(EntryError):
        EntryGeometry(
            Pose.from_xyz_rpy(), doorway_distance=0.7, distance=distance
        ).validate()


# %% native lidar geometry
@pytest.fixture
def clear_scan() -> LidarReading:
    """
    Provide finite returns beyond a doorway in a minimal sensor world.
    """
    world = World.create_with_root_body("laser")
    pattern = ScanPattern(-math.pi / 2, math.pi / 2, math.pi / 360, 0.05, 10)
    return LidarReading(world.root, pattern, np.full(pattern.beam_count, 3.0))


def test_segment_requires_measured_clearance(clear_scan: LidarReading) -> None:
    """
    Finite returns beyond the configured segment establish clearance.
    """
    start = Point3(1, -0.3, 0, reference_frame=clear_scan.reference_frame)
    end = Point3(1, 0.3, 0, reference_frame=clear_scan.reference_frame)
    assert SegmentVisibility().evaluate(clear_scan, start, end) is True


def test_missing_returns_are_unknown_not_free(clear_scan: LidarReading) -> None:
    """
    Infinite sensor ranges cannot certify an elevator opening.
    """
    clear_scan.ranges[:] = math.inf
    start = Point3(1, -0.3, 0, reference_frame=clear_scan.reference_frame)
    end = Point3(1, 0.3, 0, reference_frame=clear_scan.reference_frame)
    assert SegmentVisibility().evaluate(clear_scan, start, end) is None


def test_closed_plane_blocks_door(clear_scan: LidarReading) -> None:
    """
    The expected door plane is an obstruction until ranges pass beyond it.
    """
    clear_scan.ranges[:] = 1 / np.maximum(
        np.cos(clear_scan.scan_pattern.beam_angles), 0.001
    )
    start = Point3(1, -0.3, 0, reference_frame=clear_scan.reference_frame)
    end = Point3(1, 0.3, 0, reference_frame=clear_scan.reference_frame)
    assert SegmentVisibility().evaluate(clear_scan, start, end) is False
