"""
Authorize a first native goal without claiming an unobserved idle status.
"""

# %% imports
import pytest
from action_msgs.msg import GoalStatus, GoalStatusArray

from experiments.robocat_snm.application import EntryOptions
from experiments.robocat_snm.native_navigation import NativeNavigation, NavigationStatus

from .test_native_navigation import navigation


# %% explicitly authorized first goal
def test_bootstrap_submits_only_with_fresh_supervisor_permission(
    navigation: NativeNavigation,
) -> None:
    """
    Explicit execution and a fresh stationary/free-mux permit enable the first goal.
    """
    navigation.statuses = None
    navigation.bootstrap_navigation = True
    now = navigation.last_heartbeat
    navigation.receive_command(NavigationStatus.BOOTSTRAP_READY, now)
    navigation.tick(now, now + 10)
    navigation.client.send_goal_async.assert_called_once_with(navigation.goal)
    assert navigation.status is NavigationStatus.NAVIGATING
    assert navigation.statuses is None


@pytest.mark.parametrize("permit", ["missing", "expired", "revoked"])
def test_bootstrap_requires_current_supervisor_interlocks(
    navigation: NativeNavigation, permit: str
) -> None:
    """
    An execution flag alone cannot replace fresh stationary/free-mux evidence.
    """
    navigation.statuses = None
    navigation.bootstrap_navigation = True
    now = navigation.last_heartbeat
    if permit != "missing":
        navigation.receive_command(NavigationStatus.BOOTSTRAP_READY, now)
    if permit == "expired":
        now += navigation.bootstrap_maximum_age + 0.01
    elif permit == "revoked":
        navigation.receive_command(NavigationStatus.BOOTSTRAP_BLOCKED, now)
    navigation.tick(now, now + 10)
    navigation.client.send_goal_async.assert_not_called()
    assert navigation.status is NavigationStatus.WAITING


def test_bootstrap_never_replaces_observed_active_goal(
    navigation: NativeNavigation,
) -> None:
    """
    An observed accepted/executing/canceling goal blocks the authorized bootstrap.
    """
    navigation.bootstrap_navigation = True
    now = navigation.last_heartbeat
    navigation.receive_command(NavigationStatus.BOOTSTRAP_READY, now)
    navigation.statuses = GoalStatusArray(
        status_list=[GoalStatus(status=GoalStatus.STATUS_EXECUTING)]
    )
    navigation.tick(now, now + 10)
    navigation.client.send_goal_async.assert_not_called()
    assert navigation.status is NavigationStatus.FAILED


def test_bootstrap_does_not_bypass_missing_supervisor(
    navigation: NativeNavigation,
) -> None:
    """
    A new permit cannot revive a request whose heartbeat has expired.
    """
    navigation.statuses = None
    navigation.bootstrap_navigation = True
    now = navigation.last_heartbeat + navigation.heartbeat_timeout + 0.01
    navigation.receive_command(NavigationStatus.BOOTSTRAP_READY, now)
    navigation.tick(now, now + 10)
    navigation.client.send_goal_async.assert_not_called()
    assert navigation.stop_requested is True


def test_bootstrap_cli_requires_execute_and_preserves_read_only_override() -> None:
    """
    The dedicated authorization cannot be confused with the read-only idle flag.
    """
    options = EntryOptions.parse(
        [
            "--handover-only",
            "--execute",
            "--bootstrap-navigation",
            "--map-fingerprint",
            "checked-map",
        ]
    )
    assert options.bootstrap_navigation is True
    assert options.confirm_navigation_idle is False
    with pytest.raises(SystemExit):
        EntryOptions.parse(["--bootstrap-navigation"])
