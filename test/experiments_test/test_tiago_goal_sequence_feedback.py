# %% imports
from __future__ import annotations

import os
import select
import subprocess
from contextlib import ExitStack
from pathlib import Path

from nav_msgs.msg import Odometry

from test.experiments_test.test_tiago_giskard_safety import client_script, feedback
from test.experiments_test.test_tiago_goal_foreground import request_pipe
from test.experiments_test.test_tiago_goal_command import command


# %% measured rest, not only action cancellation
def test_settled_requires_multiple_samples_over_time(feedback, monkeypatch) -> None:
    """
    A single zero-velocity sample must not authorize the second goal.
    """
    clock = [0.0]
    monkeypatch.setattr(
        feedback.receive_wheels.__globals__["time"], "monotonic", lambda: clock[0]
    )
    message = Odometry()
    message.header.stamp.sec = 10
    feedback.receive_wheels(message)
    assert feedback.wheels_settled() is False
    clock[0] = 0.1
    feedback.receive_wheels(message)
    assert feedback.wheels_settled() is False
    clock[0] = 0.21
    feedback.receive_wheels(message)
    assert feedback.wheels_settled() is True
    message.twist.twist.linear.x = 0.1
    feedback.receive_wheels(message)
    assert feedback.wheels_settled() is False


def test_readiness_query_reports_blocked_without_motion(
    command: Path, request_pipe, client_script: dict
) -> None:
    """
    A blocked query must return its own status, not send a distance request.
    """
    path, descriptor = request_pipe
    sender = subprocess.Popen(
        ["bash", str(command), "--ready", "--fifo", str(path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    with ExitStack() as cleanup:
        cleanup.callback(sender.kill)
        assert select.select([descriptor], [], [], 3)[0] == [descriptor]
        fields = os.read(descriptor, 4096).decode().split()
        assert fields[0] == "ready"
        reply = Path(fields[2])
        parsed = client_script["GoalRequest"].parse(" ".join(fields))
        assert parsed.distance is None
        parsed.publish("blocked")
        sender.communicate(timeout=5)
        assert sender.returncode == 3
