"""
Reach staging and certify handover without submitting a Giskard motion.
"""

# %% imports
import pytest

from experiments.robocat_snm.application import EntryMission, EntryOptions
from experiments.robocat_snm.exceptions import EntryError
from experiments.robocat_snm.native_navigation import NavigationStatus
from semantic_digital_twin.spatial_types import Pose

from .test_application import mission


# %% explicit execution boundary
def test_handover_execution_requires_map_but_not_door_calibration() -> None:
    """
    Stopping at staging does not require authorizing an elevator crossing.
    """
    options = EntryOptions.parse(
        ["--handover-only", "--execute", "--map-fingerprint", "checked-map"]
    )
    assert options.handover_only is True
    assert options.execute is True
    assert options.doorway_distance is None
    assert options.doors_held_open is False


def test_handover_without_execute_remains_read_only() -> None:
    """
    Selecting the mode alone cannot authorize native movement.
    """
    options = EntryOptions.parse(["--handover-only"])
    assert options.execute is False


@pytest.mark.parametrize(
    "arguments",
    [
        ["--handover-only", "--execute"],
        ["--handover-only", "--entry-only"],
        [
            "--handover-only",
            "--execute",
            "--map-fingerprint",
            "checked-map",
            "--confirm-navigation-idle",
        ],
    ],
)
def test_handover_rejects_missing_map_or_conflicting_authorization(
    arguments: list[str],
) -> None:
    """
    Idle overrides and conflicting request modes cannot authorize the test.
    """
    with pytest.raises(SystemExit):
        EntryOptions.parse(arguments)


# %% no Giskard submission
def test_handover_finishes_after_verified_staging_without_entry(
    mission: EntryMission,
) -> None:
    """
    Native success and staging checks complete without building an entry task.
    """
    staging = Pose.from_xyz_rpy()
    mission.handover(staging)
    mission.navigation.start.assert_called_once_with()
    mission.inputs.confirm_handover.assert_called_once()
    mission.inputs.freeze.assert_not_called()
    mission.inputs.observe.assert_not_called()
    mission.pool.submit.assert_not_called()


def test_handover_rejects_changed_native_ownership(mission: EntryMission) -> None:
    """
    A new native owner after arrival cannot be reported ready for Giskard.
    """
    mission.navigation.poll.side_effect = [True, False]
    with pytest.raises(EntryError):
        mission.handover(Pose.from_xyz_rpy())
    mission.pool.submit.assert_not_called()


def test_handover_rejects_native_failure(mission: EntryMission) -> None:
    """
    A failed native navigation cannot become a certified handover.
    """
    mission.navigation.status = NavigationStatus.FAILED
    with pytest.raises(EntryError):
        mission.handover(Pose.from_xyz_rpy())
    mission.inputs.confirm_handover.assert_not_called()
    mission.pool.submit.assert_not_called()
