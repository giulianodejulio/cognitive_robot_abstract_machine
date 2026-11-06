# %% imports
from __future__ import annotations

from io import StringIO
from unittest.mock import Mock

from nav_msgs.msg import Odometry

from test.experiments_test.test_tiago_giskard_safety import client_script


# %% persistent request handling
def test_interactive_mode_does_not_immediately_execute(client_script: dict) -> None:
    """
    Starting an interactive client must not send its default distance.
    """
    options = client_script["GoalOptions"].parse(["--interactive"])
    assert options.interactive is True
    assert options.execute is False


def test_multiple_requests_reuse_one_session(client_script: dict, monkeypatch) -> None:
    """
    Dispatch successive distances through the already connected client.
    """
    session = client_script["GoalSession"](context=Mock(), feedback=Mock(), pool=Mock())
    send = Mock()
    monkeypatch.setattr(session, "send", send)
    monkeypatch.setattr(
        client_script["sys"], "stdin", StringIO("bad\n-1\n0.30\n0.50\nquit\n")
    )
    session.interact()
    assert [call.args[0] for call in send.call_args_list] == [0.3, 0.5]


# %% wheel feedback onset
def test_onset_ignores_idle_samples_and_reports_first_movement(
    client_script: dict,
) -> None:
    """
    Measure receipt latency once, without retaining a traffic history.
    """
    onset = client_script["WheelOnset"]()
    message = Odometry()
    message.twist.twist.linear.x = 0.1
    onset.observe(message, received_at=9.0)
    assert onset.latency is None
    onset.arm(10.0)
    message.twist.twist.linear.x = 0.0
    onset.observe(message, received_at=10.1)
    assert onset.latency is None
    message.twist.twist.linear.x = 0.1
    onset.observe(message, received_at=10.5)
    onset.observe(message, received_at=11.0)
    assert onset.latency == 0.5


def test_next_request_resets_onset(client_script: dict) -> None:
    """
    Do not reuse the previous goal's onset measurement.
    """
    onset = client_script["WheelOnset"]()
    message = Odometry()
    message.twist.twist.linear.x = 0.1
    onset.arm(10.0)
    onset.observe(message, received_at=10.5)
    onset.arm(20.0)
    assert onset.latency is None
    onset.observe(message, received_at=20.25)
    assert onset.latency == 0.25
