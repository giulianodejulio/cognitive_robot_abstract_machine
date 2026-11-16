"""
A rejected handover identifies the actual failed condition.
"""

# %% imports
from unittest.mock import Mock

import pytest

from experiments.robocat_snm.exceptions import EntryError
from experiments.robocat_snm.feedback import EntryInputs
from experiments.robocat_snm.geometry import StagingAlignment
from semantic_digital_twin.spatial_types import Pose

from .test_feedback import entry_inputs


# %% staging diagnostics
def test_handover_reports_numeric_position_residual(
    entry_inputs: EntryInputs, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    A centimetre-level miss reports its measured distance and configured bound.
    """
    monkeypatch.setattr(
        entry_inputs, "map_pose", Mock(return_value=Pose.from_xyz_rpy(x=0.03))
    )
    entry_inputs.feedback.ready.return_value = True
    with pytest.raises(EntryError) as caught:
        entry_inputs.confirm_handover(
            StagingAlignment(Pose.from_xyz_rpy()), entry_inputs.scene.fingerprint
        )
    assert "position error=0.0300 m" in caught.value.reason
    assert "limit=0.0200 m" in caught.value.reason


def test_handover_reports_missing_synchronized_pose(
    entry_inputs: EntryInputs, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    A missing TF cannot be confused with an out-of-tolerance position.
    """
    monkeypatch.setattr(entry_inputs, "map_pose", Mock(return_value=None))
    entry_inputs.feedback.ready.return_value = True
    with pytest.raises(EntryError) as caught:
        entry_inputs.confirm_handover(
            StagingAlignment(Pose.from_xyz_rpy()), entry_inputs.scene.fingerprint
        )
    assert "Map pose missing, stale or unsynchronized" in caught.value.reason


def test_handover_reports_actual_interlock(
    entry_inputs: EntryInputs, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    A mux or driver blocker remains distinguishable from geometry rejection.
    """
    monkeypatch.setattr(
        entry_inputs, "map_pose", Mock(return_value=Pose.from_xyz_rpy())
    )
    entry_inputs.feedback.ready.return_value = False
    entry_inputs.feedback.blocking_reasons.return_value = [
        "mux current priority=100 (expected 0)"
    ]
    with pytest.raises(EntryError) as caught:
        entry_inputs.confirm_handover(
            StagingAlignment(Pose.from_xyz_rpy()), entry_inputs.scene.fingerprint
        )
    assert entry_inputs.feedback.blocking_reasons.return_value[0] in caught.value.reason
