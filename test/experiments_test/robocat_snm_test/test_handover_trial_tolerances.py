"""
Relaxed acceptance belongs only to explicit handover-only diagnostic trials.
"""

# %% imports
import math

import pytest

from experiments.robocat_snm.application import EntryMission, EntryOptions
from experiments.robocat_snm.geometry import StagingAlignment
from semantic_digital_twin.spatial_types import Pose

from .test_application import mission


# %% diagnostic acceptance
def test_handover_trial_passes_explicit_tolerances_to_staging_check(
    mission: EntryMission,
) -> None:
    """
    An explicit diagnostic trial changes acceptance without submitting entry motion.
    """
    mission.options = EntryOptions.parse(
        [
            "--handover-only",
            "--handover-position-tolerance",
            "0.25",
            "--handover-orientation-tolerance-deg",
            "10",
        ]
    )
    staging = Pose.from_xyz_rpy()
    mission.handover(staging)
    alignment = mission.inputs.confirm_handover.call_args.args[0]
    assert alignment.position_tolerance == 0.25
    assert alignment.orientation_tolerance == math.radians(10)
    mission.pool.submit.assert_not_called()


def test_default_handover_keeps_precise_acceptance(mission: EntryMission) -> None:
    """
    Omitting diagnostic options preserves existing staging thresholds.
    """
    staging = Pose.from_xyz_rpy()
    mission.handover(staging)
    alignment = mission.inputs.confirm_handover.call_args.args[0]
    assert alignment.position_tolerance == StagingAlignment.position_tolerance
    assert alignment.orientation_tolerance == StagingAlignment.orientation_tolerance


@pytest.mark.parametrize(
    "arguments",
    [
        ["--handover-position-tolerance", "0.25"],
        ["--handover-orientation-tolerance-deg", "10"],
        ["--handover-only", "--handover-position-tolerance", "nan"],
        ["--handover-only", "--handover-position-tolerance", "0"],
        ["--handover-only", "--handover-orientation-tolerance-deg", "inf"],
        ["--handover-only", "--handover-orientation-tolerance-deg", "-1"],
    ],
)
def test_relaxed_handover_options_cannot_leak_into_entry(arguments: list[str]) -> None:
    """
    Full entry cannot inherit relaxed diagnostic thresholds or invalid limits.
    """
    with pytest.raises(SystemExit):
        EntryOptions.parse(arguments)
