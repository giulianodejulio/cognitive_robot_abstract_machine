"""
Verify owned native action lifecycle without network calls.
"""

# %% imports
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterValue
from rclpy.task import Future

from experiments.robocat_snm import native_navigation
from experiments.robocat_snm.native_navigation import NativeNavigation, NavigationStatus


# %% owned navigation
@pytest.fixture
def navigation(monkeypatch: pytest.MonkeyPatch) -> NativeNavigation:
    """
    Provide a native-ready client and a valid separate-input mux reply.
    """
    client = Mock()
    monkeypatch.setattr(native_navigation, "ActionClient", Mock(return_value=client))
    agent = NativeNavigation(Mock(), NavigateToPose.Goal(), execute=True)
    agent.statuses = GoalStatusArray()
    reply = Future()
    reply.set_result(
        Mock(
            values=[
                ParameterValue(string_value="cmd_vel"),
                ParameterValue(integer_value=10),
                ParameterValue(string_value="snm_vel"),
                ParameterValue(integer_value=20),
            ]
        )
    )
    agent.routing = reply
    return agent


def test_foreign_active_goal_is_not_replaced(navigation: NativeNavigation) -> None:
    """
    A foreign active goal rejects this request without a cancel-all operation.
    """
    navigation.statuses.status_list = [GoalStatus(status=GoalStatus.STATUS_EXECUTING)]
    navigation.tick(navigation.last_heartbeat, navigation.last_heartbeat + 10)
    assert navigation.status is NavigationStatus.FAILED
    navigation.client.send_goal_async.assert_not_called()

def test_manual_confirmation_does_not_override_active_goal(
    navigation: NativeNavigation,
) -> None:
    navigation.execute = False
    navigation.confirm_navigation_idle = True
    navigation.statuses.status_list = [
        GoalStatus(status=GoalStatus.STATUS_EXECUTING)
    ]

    navigation.tick(
        navigation.last_heartbeat,
        navigation.last_heartbeat + 10,
    )

    assert navigation.status is NavigationStatus.FAILED
    navigation.client.send_goal_async.assert_not_called()

def test_late_acceptance_after_cancel_is_still_canceled(
    navigation: NativeNavigation,
) -> None:
    """
    Cancellation survives a send-goal request whose acceptance was delayed.
    """
    navigation.status = NavigationStatus.NAVIGATING
    navigation.pending = Future()
    navigation.stop_requested = True
    handle = Mock(accepted=True)
    handle.get_result_async.return_value = Future()
    navigation.pending.set_result(handle)
    navigation.tick(navigation.last_heartbeat, navigation.last_heartbeat + 10)
    handle.cancel_goal_async.assert_called_once_with()


def test_unsuccessful_nav2_result_cannot_trigger_handover(
    navigation: NativeNavigation,
) -> None:
    """
    A terminal result is not sufficient unless its status is succeeded.
    """
    navigation.status = NavigationStatus.NAVIGATING
    navigation.result = Future()
    navigation.result.set_result(Mock(status=GoalStatus.STATUS_ABORTED))
    navigation.tick(navigation.last_heartbeat, navigation.last_heartbeat + 10)
    assert navigation.status is NavigationStatus.FAILED


def test_missing_supervisor_cancels_owned_goal(navigation: NativeNavigation) -> None:
    """
    The native watchdog requests cancellation when supervisor heartbeats stop.
    """
    navigation.status = NavigationStatus.NAVIGATING
    navigation.handle = Mock()
    navigation.tick(
        navigation.last_heartbeat + navigation.heartbeat_timeout + 0.1, float("inf")
    )
    navigation.handle.cancel_goal_async.assert_called_once_with()


def test_cleanup_drains_late_acceptance_before_ros_shutdown(
    navigation: NativeNavigation, monkeypatch: pytest.MonkeyPatch
) -> None:
    """
    Output failure cleanup must still accept and cancel the owned pending goal.
    """
    handle = Mock(accepted=True)
    result = Future()
    handle.get_result_async.return_value = result
    navigation.pending = Future()
    navigation.pending.set_result(handle)
    navigation.status = NavigationStatus.NAVIGATING

    def complete_cancel(node, timeout_sec) -> None:
        """
        Acknowledge only after the owned cancellation request has been sent.
        """
        if navigation.cancellation is not None and not result.done():
            result.set_result(Mock(status=GoalStatus.STATUS_CANCELED))

    monkeypatch.setattr(native_navigation.rclpy, "ok", lambda: True)
    monkeypatch.setattr(native_navigation.rclpy, "spin_once", complete_cancel)
    navigation.close()
    handle.cancel_goal_async.assert_called_once_with()
    assert navigation.status is NavigationStatus.CANCELED

def test_manual_confirmation_without_status(navigation: NativeNavigation) -> None:
    navigation.execute = False
    navigation.confirm_navigation_idle = True
    navigation.statuses = None
    navigation.tick(
        navigation.last_heartbeat,
        navigation.last_heartbeat + 10,
    )

    assert navigation.status is NavigationStatus.SUCCEEDED
    navigation.client.send_goal_async.assert_not_called()

def test_missing_status_without_confirmation_blocks(
    navigation: NativeNavigation,
) -> None:
    navigation.execute = False
    navigation.confirm_navigation_idle = False
    navigation.statuses = None

    navigation.tick(
        navigation.last_heartbeat,
        navigation.last_heartbeat + 10,
    )

    assert navigation.status is NavigationStatus.WAITING
    navigation.client.send_goal_async.assert_not_called()

def test_interrupt_requests_owned_goal_cancellation(
    navigation: NativeNavigation,
) -> None:
    navigation.status = NavigationStatus.NAVIGATING
    navigation.handle = Mock()
    navigation.stop_requested = True

    navigation.tick(
        navigation.last_heartbeat,
        navigation.last_heartbeat + 10,
    )

    navigation.handle.cancel_goal_async.assert_called_once_with()