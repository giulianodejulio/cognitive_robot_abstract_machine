"""
Only the requested native map waypoint can become an entry target.
"""

# %% imports
from unittest.mock import Mock
import math
import subprocess
from pathlib import Path

import pytest
from geometry_msgs.msg import Pose as GeometryPose
from experiments.robocat_snm.exceptions import EntryError
from experiments.robocat_snm.waypoint import (
    SavedWaypoint,
    WaypointConnection,
    WaypointReply,
)


# %% native coordinate validation
@pytest.mark.parametrize(
    "key,frame,rotation,valid",
    [
        ("floor/inside", "map", 1.0, True),
        ("floor/other", "map", 1.0, False),
        ("floor/inside", "odom", 1.0, False),
        ("floor/inside", "map", 0.0, False),
        ("floor/inside", "map", math.nan, False),
    ],
)
def test_reply_requires_requested_map_identity(
    key: str, frame: str, rotation: float, valid: bool
) -> None:
    """
    Wrong frames and invalid quaternions fail before any motion is created.
    """
    pose = GeometryPose()
    pose.position.x = 1.3
    pose.orientation.w = rotation
    saved = SavedWaypoint(key, frame, pose)
    if valid:
        assert float(saved.in_map("floor/inside").x) == 1.3
    else:
        with pytest.raises(EntryError):
            saved.in_map("floor/inside")


def test_foreign_floor_does_not_open_ssh(monkeypatch) -> None:
    """
    Inside and staging must refer to the same Eulero floor.
    """
    run = Mock()
    monkeypatch.setattr(subprocess, "run", run)
    with pytest.raises(EntryError):
        WaypointConnection().read("other_floor/inside", "floor")
    run.assert_not_called()


def test_native_transport_failure_is_not_a_target(monkeypatch) -> None:
    """
    An empty or failed SSH reply cannot silently fall back to distance.
    """
    monkeypatch.setattr(Path, "is_socket", lambda path: True)
    monkeypatch.setattr(
        subprocess,
        "run",
        Mock(return_value=subprocess.CompletedProcess([], 1, "", "query failed")),
    )
    with pytest.raises(EntryError):
        WaypointConnection().read("inside", "floor")
