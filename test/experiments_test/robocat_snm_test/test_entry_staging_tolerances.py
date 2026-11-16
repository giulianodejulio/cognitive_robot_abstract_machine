"""
Entry acceptance is configurable independently of its final motion accuracy.
"""

# %% imports
import math

import pytest

from experiments.robocat_snm.application import EntryOptions
from experiments.robocat_snm.geometry import EntryGeometry
from semantic_digital_twin.spatial_types import Pose


# %% independent staging acceptance
def test_entry_staging_acceptance_can_be_relaxed_without_relaxing_final_goal() -> None:
    """
    A measured staging residual can be accepted while preserving precise entry
    completion.
    """
    options = EntryOptions.parse(
        [
            "--entry-only",
            "--staging-position-tolerance",
            "0.05",
            "--staging-orientation-tolerance-deg",
            "10",
        ]
    )
    geometry = options.entry_geometry(Pose.from_xyz_rpy(), doorway_distance=0.7)
    assert geometry.aligned(Pose.from_xyz_rpy(x=0.03, yaw=math.radians(8))) is True
    assert (
        geometry.reached(Pose.from_xyz_rpy(x=geometry.distance, yaw=math.radians(8)))
        is False
    )
    assert geometry.position_tolerance == EntryGeometry.position_tolerance
    assert geometry.orientation_tolerance == EntryGeometry.orientation_tolerance


@pytest.mark.parametrize(
    "arguments",
    [
        ["--staging-position-tolerance", "nan"],
        ["--staging-position-tolerance", "0"],
        ["--staging-orientation-tolerance-deg", "inf"],
        ["--staging-orientation-tolerance-deg", "-1"],
    ],
)
def test_entry_staging_acceptance_requires_finite_positive_limits(
    arguments: list[str],
) -> None:
    """
    Invalid acceptance bounds never reach an entry action.
    """
    with pytest.raises(SystemExit):
        EntryOptions.parse(arguments)
