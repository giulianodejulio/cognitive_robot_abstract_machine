"""
Recorded cabin waypoints replace distance-only entry targets.
"""

# %% imports
import math
from dataclasses import replace

import pytest

from experiments.robocat_snm.application import EntryOptions
from experiments.robocat_snm.geometry import EntryGeometry
from experiments.robocat_snm.exceptions import EntryError
from experiments.robocat_waypoints.model import WaypointSnapshot
from semantic_digital_twin.spatial_types import Pose


# %% independent waypoint coordinates
def test_inside_waypoint_is_preserved_in_staging_coordinates() -> None:
    """
    A recorded lateral displacement and heading are not silently discarded.
    """
    staging = Pose.from_xyz_rpy(x=4, y=5, yaw=math.pi / 2)
    inside = Pose.from_xyz_rpy(x=4.04, y=6.4, yaw=math.pi / 2 + 0.01)
    options = EntryOptions.parse(["--inside-waypoint", "elevator_inside"])
    geometry = options.entry_geometry(staging, 0.86, inside=inside)
    assert float(geometry.target.x) == pytest.approx(float(inside.x))
    assert float(geometry.target.y) == pytest.approx(float(inside.y))
    assert float(geometry.target.yaw) == pytest.approx(float(inside.yaw))
    assert geometry.distance == pytest.approx(1.4)
    assert geometry.reached(inside) is True
    assert replace(
        geometry, staging=Pose.from_xyz_rpy()
    ).target_lateral == pytest.approx(-0.04)


def test_distance_and_inside_waypoint_are_mutually_exclusive() -> None:
    """
    A request cannot hide which target definition is used.
    """
    with pytest.raises(SystemExit):
        EntryOptions.parse(
            ["--distance", "1.3", "--inside-waypoint", "elevator_inside"]
        )


@pytest.mark.parametrize(
    "inside",
    [
        Pose.from_xyz_rpy(x=0.8),
        Pose.from_xyz_rpy(x=1.4, y=0.3),
        Pose.from_xyz_rpy(x=1.4, yaw=0.3),
    ],
)
def test_inside_waypoint_must_fit_a_straight_checked_entry(inside: Pose) -> None:
    """
    Waypoints on the threshold, outside the corridor or misaligned are rejected.
    """
    options = EntryOptions(inside_waypoint="elevator_inside")
    with pytest.raises(EntryError):
        options.entry_geometry(Pose.from_xyz_rpy(), 0.86, inside=inside).validate()


def test_waypoint_offsets_survive_odometry_freeze() -> None:
    """
    The same map waypoint is used by motion and final odometric assessment.
    """
    staging = Pose.from_xyz_rpy(x=4, y=5, yaw=math.pi / 2)
    inside = Pose.from_xyz_rpy(x=4.04, y=6.4, yaw=math.pi / 2 + 0.01)
    options = EntryOptions(inside_waypoint="elevator_inside")
    geometry = options.entry_geometry(staging, 0.86, inside=inside)
    frozen = replace(geometry, staging=Pose.from_xyz_rpy(x=1, y=2, yaw=0.2))
    snapshot = WaypointSnapshot(staging, frozen.staging, inside)
    assert float(frozen.target.x) == pytest.approx(float(snapshot.feedback_target.x))
    assert float(frozen.target.y) == pytest.approx(float(snapshot.feedback_target.y))
    assert float(frozen.target.yaw) == pytest.approx(
        float(snapshot.feedback_target.yaw)
    )
    assert frozen.assess(snapshot.feedback_target).target_reached is True


def test_final_tolerance_can_be_explicitly_relaxed_for_waypoint_trial() -> None:
    """
    Five centimetres can be requested without changing wheel dynamics.
    """
    options = EntryOptions.parse(
        ["--inside-waypoint", "elevator_inside", "--position-tolerance", "0.05"]
    )
    inside = Pose.from_xyz_rpy(x=1.4)
    geometry = options.entry_geometry(Pose.from_xyz_rpy(), 0.86, inside=inside)
    assert geometry.position_tolerance == 0.05
    assert geometry.reached(Pose.from_xyz_rpy(x=1.36)) is True
    assert geometry.reached(Pose.from_xyz_rpy(x=1.34)) is False
