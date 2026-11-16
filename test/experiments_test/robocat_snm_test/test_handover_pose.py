"""
Require a stopped, localized base at the validated staging waypoint.
"""

# %% imports
from unittest.mock import Mock

import pytest

from experiments.robocat_snm.exceptions import EntryError
from experiments.robocat_snm.feedback import EntryInputs
from experiments.robocat_snm.geometry import StagingAlignment
from semantic_digital_twin.spatial_types import Pose

from .test_feedback import entry_inputs


# %% reached staging evidence
def test_handover_accepts_current_stopped_staging(
    entry_inputs: EntryInputs, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    The actual staging pose is returned only with fresh released interlocks.
    """
    pose = Pose.from_xyz_rpy(x=4, y=2)
    monkeypatch.setattr(entry_inputs, "map_pose", Mock(return_value=pose))
    entry_inputs.feedback.ready.return_value = True
    result = entry_inputs.confirm_handover(
        StagingAlignment(pose), entry_inputs.scene.fingerprint
    )
    assert result is pose
    entry_inputs.feedback.ready.assert_called_once_with()


@pytest.mark.parametrize(
    "fault", ["map", "pose", "position", "orientation", "interlock", "moving"]
)
def test_handover_rejects_incomplete_staging_evidence(
    entry_inputs: EntryInputs, monkeypatch: pytest.MonkeyPatch, fault: str
) -> None:
    """
    Map, pose, alignment, operator interlocks and measured arrest are mandatory.
    """
    staging = Pose.from_xyz_rpy()
    alignment = StagingAlignment(staging)
    pose = staging
    fingerprint = entry_inputs.scene.fingerprint
    entry_inputs.feedback.ready.return_value = True
    if fault == "map":
        fingerprint = "different-map"
    elif fault == "pose":
        pose = None
    elif fault == "position":
        pose = Pose.from_xyz_rpy(x=alignment.position_tolerance * 2)
    elif fault == "orientation":
        pose = Pose.from_xyz_rpy(yaw=alignment.orientation_tolerance * 2)
    elif fault == "interlock":
        entry_inputs.feedback.ready.return_value = False
    elif fault == "moving":
        entry_inputs.feedback.wheels_settled.return_value = False
    monkeypatch.setattr(entry_inputs, "map_pose", Mock(return_value=pose))
    with pytest.raises(EntryError):
        entry_inputs.confirm_handover(alignment, fingerprint)
