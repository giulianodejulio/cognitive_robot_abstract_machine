# %% imports
from __future__ import annotations

import importlib
import math
from concurrent.futures import Future
from contextlib import nullcontext
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Odometry

from test.experiments_test.test_tiago_giskard_safety import client_script


# %% signed errors in the initial robot axes
@pytest.fixture
def error_logging():
    """
    Load the logger without initializing any ROS node or worker.
    """
    return importlib.import_module("experiments.tiago_goal_logging")


def test_errors_are_projected_in_the_initial_heading(error_logging) -> None:
    """
    The same forward request must work when odom axes differ from robot axes.
    """
    start = error_logging.PlanarPose(x=2, y=-1, yaw=math.pi / 2)
    logger = error_logging.GoalErrorLog(start=start, distance=1, requested_at=0)
    errors = logger.measure(
        error_logging.PlanarPose(x=2.03, y=-0.2, yaw=start.yaw + math.radians(3))
    )
    assert errors.longitudinal == pytest.approx(0.2)
    assert errors.lateral == pytest.approx(0.03)
    assert errors.angular_degrees == pytest.approx(-3)


def test_angular_error_uses_shortest_distance(error_logging) -> None:
    """
    A yaw wrap must not look like an extra full revolution.
    """
    start = error_logging.PlanarPose(x=0, y=0, yaw=math.radians(179))
    logger = error_logging.GoalErrorLog(
        start=start, distance=0, turn_degrees=30, requested_at=0
    )
    errors = logger.measure(error_logging.PlanarPose(x=0, y=0, yaw=math.radians(-152)))
    assert errors.angular_degrees == pytest.approx(1)


def test_periodic_logging_is_limited_to_one_line_per_second(
    error_logging, monkeypatch, capsys
) -> None:
    """
    Polling the latest sample must not flood logs or add an independent timer.
    """
    clock = [0.0]
    monkeypatch.setattr(error_logging.time, "monotonic", lambda: clock[0])
    logger = error_logging.GoalErrorLog(
        start=error_logging.PlanarPose(0, 0, 0), distance=1, requested_at=0
    )
    message = Odometry()
    message.pose.pose.orientation.w = 1.0
    logger.report(message, error_logging.GoalErrorPhase.TRACKING)
    clock[0] = 0.2
    logger.report(message, error_logging.GoalErrorPhase.TRACKING)
    clock[0] = 1.0
    logger.report(message, error_logging.GoalErrorPhase.TRACKING)
    lines = capsys.readouterr().out.splitlines()
    assert len(lines) == 2
    assert all("longitudinal=+1.0000 m" in line for line in lines)


def test_result_snapshot_is_not_throttled(error_logging, monkeypatch, capsys) -> None:
    """
    Failure or success must retain an error snapshot even between periodic lines.
    """
    monkeypatch.setattr(error_logging.time, "monotonic", lambda: 0.0)
    logger = error_logging.GoalErrorLog(
        start=error_logging.PlanarPose(0, 0, 0), distance=1, requested_at=0
    )
    message = Odometry()
    message.pose.pose.orientation.w = 1.0
    logger.report(message, error_logging.GoalErrorPhase.TRACKING)
    logger.report(message, error_logging.GoalErrorPhase.RESULT, force=True)
    lines = capsys.readouterr().out.splitlines()
    assert len(lines) == 2
    assert "phase=result" in lines[1]


# %% failure logging through the existing session
def test_failed_goal_logs_errors_before_returning(
    client_script: dict, monkeypatch, capsys
) -> None:
    """
    Aborted actions must report errors even though their settled summary is skipped.
    """
    session_type = client_script["GoalSession"]
    namespace = session_type.send.__globals__
    monkeypatch.setitem(namespace, "execute_single", Mock())
    monkeypatch.setitem(namespace, "real_robot", lambda **kwargs: nullcontext())
    feedback = Mock()
    feedback.odometry = Odometry()
    feedback.odometry.pose.pose.orientation.w = 1.0
    completed = Future()
    completed.set_exception(RuntimeError("test action failure"))
    pool = Mock()
    pool.submit.return_value = completed
    session = session_type(context=Mock(), feedback=feedback, pool=pool)
    assert session.send(0.4) is False
    output = capsys.readouterr().out
    assert "GOAL ERROR: phase=result" in output
    assert "longitudinal=+0.4000 m" in output
    assert output.index("GOAL ERROR:") < output.index("GOAL FAILED:")
