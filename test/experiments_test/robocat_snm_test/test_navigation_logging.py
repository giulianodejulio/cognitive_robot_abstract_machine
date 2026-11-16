"""
Navigation heartbeats update supervision without repeating terminal messages.
"""

# %% imports
from unittest.mock import Mock

from experiments.robocat_snm import navigation
from experiments.robocat_snm.native_navigation import NavigationStatus
from experiments.robocat_snm.navigation import NavigationConnection
from semantic_digital_twin.spatial_types import Pose


# %% terminal output
def test_poll_logs_lifecycle_changes_without_heartbeat_noise(
    monkeypatch, capsys
) -> None:
    """
    Repeated routing and ownership evidence remains effective but silent.
    """
    connection = NavigationConnection(Pose.from_xyz_rpy())
    connection.process = Mock()
    connection.process.poll.return_value = None
    statuses = [
        NavigationStatus.NAVIGATING,
        NavigationStatus.BUSY,
        NavigationStatus.ROUTING_READY,
        NavigationStatus.NAVIGATING,
        NavigationStatus.SUCCEEDED,
        NavigationStatus.IDLE,
        NavigationStatus.ROUTING_READY,
        NavigationStatus.SUCCEEDED,
    ]
    monkeypatch.setattr(
        navigation.select, "select", Mock(side_effect=[([True], [], []), ([], [], [])])
    )
    monkeypatch.setattr(
        navigation.os, "read", Mock(return_value=("\n".join(statuses) + "\n").encode())
    )
    assert connection.poll() is True
    assert capsys.readouterr().out.splitlines() == [
        f"NATIVE STATUS: {NavigationStatus.NAVIGATING.value!r}",
        f"NATIVE STATUS: {NavigationStatus.SUCCEEDED.value!r}",
    ]
    assert connection.idle is True
    assert connection.routing_ready is True
    assert connection.status is NavigationStatus.SUCCEEDED
