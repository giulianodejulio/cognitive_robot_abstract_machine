# %% imports
from __future__ import annotations

import runpy
from concurrent.futures import Future
from pathlib import Path
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


# %% real motion preconditions
@pytest.fixture(scope="module")
def client_script() -> dict:
    """
    Load the client without creating a ROS node.
    """
    return runpy.run_path(
        str(Path(__file__).parents[2] / "scripts/tiago_giskard_goal.py")
    )


@pytest.fixture
def feedback(client_script: dict):
    """
    Provide stationary, current sensor samples with released interlocks.
    """
    node = Mock()
    node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
    feedback = client_script["BaseFeedback"](node)
    feedback.receive_emergency(Bool(data=False))
    feedback.receive_joy(Bool(data=False))
    odometry = Odometry()
    odometry.header.stamp.sec = 10
    odometry.pose.pose.orientation.w = 1.0
    feedback.receive_odometry(odometry)
    joints = JointState(position=[0.0])
    joints.header.stamp.sec = 10
    feedback.receive_joints(joints)
    feedback.mux["current priority"] = "0"
    return feedback


def test_emergency_prevents_goal_and_requests_running_goal_cancellation(
    feedback,
) -> None:
    """
    An asserted emergency invalidates both preflight and running feedback.
    """
    assert feedback.ready() is True
    feedback.receive_emergency(Bool(data=True))
    assert feedback.ready() is False
    assert feedback.ready(require_free_mux=False) is False


def test_stale_odometry_invalidates_feedback(feedback) -> None:
    """
    Stop trusting the pose when sensor timestamps cease advancing.
    """
    feedback.odometry.header.stamp.sec = 7
    assert feedback.ready(require_free_mux=False) is False


def test_active_navigation_does_not_cancel_itself(feedback) -> None:
    """
    A navigation-owned mux prevents a new goal but permits the running goal.
    """
    feedback.mux["current priority"] = "10"
    assert feedback.ready() is False
    assert feedback.ready(require_free_mux=False) is True


def test_cancel_only_owned_running_goal_once(client_script: dict) -> None:
    """
    Repeated timeout checks issue only one cancellation for the owned action.
    """
    wrapper = Mock()
    execution = Future()
    cancellation = client_script["GoalCancellation"](wrapper, execution)
    cancellation.cancel()
    cancellation.cancel()
    wrapper.cancel_goal_async.assert_called_once_with()


def test_completed_goal_is_not_canceled(client_script: dict) -> None:
    """
    Closing a successful client leaves the persistent server idle.
    """
    wrapper = Mock()
    execution = Future()
    execution.set_result(None)
    client_script["GoalCancellation"](wrapper, execution).cancel()
    wrapper.cancel_goal_async.assert_not_called()
