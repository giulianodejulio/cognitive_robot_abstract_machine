"""
A bootstrap permit is current control evidence, never a fabricated idle status.
"""

# %% imports
import io
import os
from unittest.mock import Mock

import pytest

from experiments.robocat_snm import native_navigation
from experiments.robocat_snm.application import EntryMission
from experiments.robocat_snm.exceptions import EntryError
from experiments.robocat_snm.native_navigation import NativeNavigation, NavigationStatus
from experiments.robocat_snm.navigation import NavigationConnection
from semantic_digital_twin.spatial_types import Pose

from .test_application import mission
from .test_native_navigation import navigation


# %% supervisor evidence
def test_approach_sends_bootstrap_permit_only_for_stopped_free_base(
    mission: EntryMission,
) -> None:
    """
    The existing measured arrest and full preflight authorize the native permit.
    """
    mission.options.bootstrap_navigation = True
    mission.approach()
    mission.navigation.poll.assert_called_once_with(bootstrap_ready=True)


@pytest.mark.parametrize("blocker", ["mux", "moving"])
def test_approach_withholds_bootstrap_when_base_not_free_and_stopped(
    mission: EntryMission, blocker: str
) -> None:
    """
    An allowed active priority cannot pass the stationary bootstrap preflight.
    """
    mission.options.bootstrap_navigation = True
    mission.navigation.status = NavigationStatus.FAILED
    if blocker == "mux":
        mission.inputs.feedback.ready.side_effect = (
            lambda require_free_mux=True: not require_free_mux
        )
    else:
        mission.inputs.feedback.wheels_settled.return_value = False
    with pytest.raises(EntryError):
        mission.approach()
    mission.navigation.poll.assert_called_once_with(bootstrap_ready=False)
    mission.pool.submit.assert_not_called()


def test_transport_sends_and_revokes_bootstrap_without_claiming_idle() -> None:
    """
    Permissions travel with heartbeats while the native status remains unknown.
    """
    connection = NavigationConnection(
        Pose.from_xyz_rpy(), execute=True, bootstrap_navigation=True
    )
    process = Mock()
    process.poll.return_value = None
    process.stdin = io.BytesIO()
    connection.process = process
    read_descriptor, write_descriptor = os.pipe()
    with os.fdopen(read_descriptor, "rb") as output, os.fdopen(write_descriptor, "wb"):
        process.stdout = output
        assert connection.poll(bootstrap_ready=True) is False
        assert process.stdin.getvalue().decode().splitlines() == [
            NavigationStatus.HEARTBEAT.value,
            NavigationStatus.BOOTSTRAP_READY.value,
        ]
        process.stdin.seek(0)
        process.stdin.truncate()
        assert connection.poll(bootstrap_ready=False) is False
        assert process.stdin.getvalue().decode().splitlines() == [
            NavigationStatus.HEARTBEAT.value,
            NavigationStatus.BOOTSTRAP_BLOCKED.value,
        ]
        assert connection.idle is False


def test_native_bootstrap_does_not_report_unobserved_idle(
    navigation: NativeNavigation,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """
    Starting the first goal never fabricates the status needed for Giskard handover.
    """
    navigation.statuses = None
    navigation.bootstrap_navigation = True
    navigation.receive_command(
        NavigationStatus.BOOTSTRAP_READY, navigation.last_heartbeat
    )
    monkeypatch.setattr(native_navigation.rclpy, "ok", Mock(side_effect=[True, False]))
    monkeypatch.setattr(native_navigation.rclpy, "spin_once", Mock())
    monkeypatch.setattr(
        native_navigation.select, "select", Mock(return_value=([], [], []))
    )
    navigation.run()
    assert NavigationStatus.IDLE.value not in capsys.readouterr().out.splitlines()
    navigation.client.send_goal_async.assert_called_once_with(navigation.goal)
