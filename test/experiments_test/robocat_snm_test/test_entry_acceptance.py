"""
Independent final bounds for straight elevator entry.
"""

# %% imports
import pytest
from experiments.robocat_snm.geometry import EntryGeometry
from semantic_digital_twin.spatial_types import Pose


# %% recorded lateral residual
@pytest.mark.parametrize(
    "x,y,accepted", [(1.30, 0.0261, True), (1.30, 0.051, False), (1.36, 0.0, False)]
)
def test_final_entry_accepts_only_configured_axis_bounds(
    x: float, y: float, accepted: bool
) -> None:
    """
    A lateral offset need not cause longitudinal recovery commands.
    """
    geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.86, distance=1.30)
    assert geometry.reached(Pose.from_xyz_rpy(x=x, y=y)) is accepted


def test_relaxed_precision_does_not_certify_rear_on_threshold() -> None:
    """
    Target precision cannot replace complete chassis clearance.
    """
    geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.86, distance=1.30)
    assert geometry.contains_robot(Pose.from_xyz_rpy(x=1.25)) is False


@pytest.mark.parametrize("yaw", [0.0, 1.57])
def test_complete_chassis_must_fit_checked_corridor(yaw: float) -> None:
    """
    Rear clearance alone cannot certify a chassis outside corridor sides.
    """
    geometry = EntryGeometry(Pose.from_xyz_rpy(), doorway_distance=0.86, distance=1.5)
    assert geometry.contains_robot(Pose.from_xyz_rpy(x=1.5, y=0.12, yaw=yaw)) is False
