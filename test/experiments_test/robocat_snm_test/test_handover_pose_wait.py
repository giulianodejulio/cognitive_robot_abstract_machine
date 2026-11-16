"""
A transient unavailable TF after Nav2 success should be reacquired while stopped.
"""

# %% imports
from unittest.mock import Mock

import pytest

from experiments.robocat_snm import feedback
from experiments.robocat_snm.feedback import EntryInputs
from experiments.robocat_snm.exceptions import EntryError
from experiments.robocat_snm.geometry import StagingAlignment
from semantic_digital_twin.spatial_types import Pose

from .test_feedback import entry_inputs


# %% synchronized handover
def test_handover_waits_for_a_current_pose_without_relaxing_limits(
    entry_inputs: EntryInputs, monkeypatch
) -> None:
    """
    One missed transform is retried before rejecting an otherwise valid stopped
    handover.
    """
    pose = Pose.from_xyz_rpy()
    monkeypatch.setattr(entry_inputs, "map_pose", Mock(side_effect=[None, pose]))
    monkeypatch.setattr(feedback.time, "sleep", Mock())
    entry_inputs.feedback.ready.return_value = True
    result = entry_inputs.confirm_handover(
        StagingAlignment(pose), entry_inputs.scene.fingerprint
    )
    assert result is pose
    assert entry_inputs.map_pose.call_count == 2


def test_handover_stops_waiting_if_an_interlock_changes(
    entry_inputs: EntryInputs, monkeypatch
) -> None:
    """
    An operator or driver interlock cannot be ignored during pose reacquisition.
    """
    monkeypatch.setattr(entry_inputs, "map_pose", Mock(return_value=None))
    entry_inputs.feedback.ready.return_value = False
    entry_inputs.feedback.blocking_reasons.return_value = ["driver fault"]
    with pytest.raises(EntryError):
        entry_inputs.confirm_handover(
            StagingAlignment(Pose.from_xyz_rpy()), entry_inputs.scene.fingerprint
        )
    assert entry_inputs.map_pose.call_count == 1


def test_handover_does_not_wait_indefinitely_for_a_missing_pose(
    entry_inputs: EntryInputs, monkeypatch
) -> None:
    """
    A persistent TF outage remains an error with a bounded wait.
    """
    monkeypatch.setattr(entry_inputs, "map_pose", Mock(return_value=None))
    entry_inputs.feedback.ready.return_value = True
    monkeypatch.setattr(feedback.time, "monotonic", Mock(side_effect=[0.0, 1.0]))
    with pytest.raises(EntryError):
        entry_inputs.confirm_handover(
            StagingAlignment(Pose.from_xyz_rpy()), entry_inputs.scene.fingerprint
        )
