"""
Apply the requested two-centimetre staging acceptance bound.
"""

# %% imports
import math

import pytest

from experiments.robocat_snm.geometry import EntryGeometry, StagingAlignment
from semantic_digital_twin.spatial_types import Pose


# %% staging acceptance
@pytest.mark.parametrize(
    "x,y,accepted",
    [
        (0.019, 0.0, True),
        (0.021, 0.0, False),
        (0.0, 0.021, False),
        (0.015, 0.015, False),
    ],
)
def test_staging_rejects_position_error_over_two_centimetres(
    x: float, y: float, accepted: bool
) -> None:
    """
    Both handover modes apply the planar distance bound, including diagonal error.
    """
    staging = Pose.from_xyz_rpy()
    reached = Pose.from_xyz_rpy(x=x, y=y)
    alignment = StagingAlignment(staging)
    geometry = EntryGeometry(staging, doorway_distance=0.7)
    assert alignment.aligned(reached) is accepted
    assert geometry.aligned(reached) is accepted


def test_staging_defaults_keep_five_degree_heading_bound() -> None:
    """
    The position change leaves the orientation acceptance unchanged.
    """
    alignment = StagingAlignment(Pose.from_xyz_rpy())
    assert alignment.position_tolerance == 0.02
    assert alignment.orientation_tolerance == math.radians(5)
