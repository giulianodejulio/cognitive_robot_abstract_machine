# %% imports
from __future__ import annotations

from concurrent.futures import Future
from contextlib import nullcontext
from unittest.mock import Mock

from nav_msgs.msg import Odometry

from test.experiments_test.test_tiago_giskard_safety import client_script


# %% client event timestamps without movement
def test_receipt_and_success_are_logged_before_settling_delay(
    client_script: dict, monkeypatch, capsys
) -> None:
    """
    Success time must describe the result, not the later final pose sample.
    """
    session_type = client_script["GoalSession"]
    globals = session_type.send.__globals__
    monkeypatch.setitem(globals, "real_robot", lambda **kwargs: nullcontext())
    monkeypatch.setitem(globals, "execute_single", Mock())
    monkeypatch.setitem(globals, "Pose", Mock())
    monkeypatch.setattr(globals["time"], "time", lambda: 123.0)
    monkeypatch.setattr(globals["time"], "monotonic", lambda: 7.0)
    before_settling = []
    monkeypatch.setattr(
        globals["time"],
        "sleep",
        lambda seconds: before_settling.append(capsys.readouterr().out),
    )
    feedback = Mock()
    feedback.odometry = Odometry()
    feedback.onset.latency = None
    execution = Future()
    execution.set_result(None)
    pool = Mock()
    pool.submit.return_value = execution
    session = session_type(context=Mock(), feedback=feedback, pool=pool)
    assert session.send(0.3, requested_at=5.0) is True
    assert "CLIENT RECEIVED: distance=0.3 m; time=123.000000" in before_settling[0]
    assert "SUCCESS: time=123.000000; elapsed=2.000 s" in before_settling[0]
