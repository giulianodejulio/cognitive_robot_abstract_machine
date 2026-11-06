# %% imports
from __future__ import annotations

from std_msgs.msg import Bool

from test.experiments_test.test_tiago_giskard_safety import client_script, feedback


# %% timestamp and blocking-state regressions
def test_small_future_timestamp_matches_server_tolerance(feedback) -> None:
    """
    Accept the same bounded clock offset as the server feedback guard.
    """
    feedback.odometry.header.stamp.nanosec = 1_000_000
    assert feedback.ready() is True


def test_excessive_future_timestamp_is_rejected(feedback) -> None:
    """
    Reject a large clock offset instead of masking a synchronization fault.
    """
    feedback.odometry.header.stamp.sec = 13
    assert feedback.ready() is False


def test_report_joystick_and_charging_locks(feedback) -> None:
    """
    Identify each observed blocker instead of returning a generic failure.
    """
    feedback.receive_joy(Bool(data=True))
    feedback.mux.update({"current priority": "210", "lock locks.charging": " locked"})
    assert feedback.blocking_reasons() == [
        "joy_priority=true",
        "mux current priority=210 (expected 0)",
        "lock locks.charging: locked",
    ]


def test_check_mode_does_not_require_execution_opt_in(client_script: dict) -> None:
    """
    Allow a read-only preflight without authorization to send a goal.
    """
    options = client_script["GoalOptions"].parse(["--check"])
    assert options.check is True
    assert options.execute is False
