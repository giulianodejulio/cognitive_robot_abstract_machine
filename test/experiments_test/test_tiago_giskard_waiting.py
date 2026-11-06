# %% imports
from __future__ import annotations

from io import StringIO
from pathlib import Path
from unittest.mock import Mock

from std_msgs.msg import Bool

from test.experiments_test.test_tiago_giskard_safety import client_script, feedback


# %% persistent client interlocks
def test_persistent_startup_accepts_blocked_interlocks(client_script: dict) -> None:
    """
    Connection startup must not depend on permission to move.
    """
    options = client_script["GoalOptions"](listen=Path("/tmp/test-only.fifo"))
    feedback = Mock()
    feedback.ready.return_value = False
    feedback.blocking_reasons.return_value = ["joy_priority=true"]
    options.preflight(feedback)


def test_blocked_request_is_rejected_without_ending_session(
    client_script: dict, feedback
) -> None:
    """
    A joystick lock rejects this request without starting an action worker.
    """
    feedback.receive_joy(Bool(data=True))
    context = Mock()
    pool = Mock()
    session = client_script["GoalSession"](
        context=context, feedback=feedback, pool=pool
    )
    assert session.send(0.3) is False
    pool.submit.assert_not_called()


def test_client_keeps_reading_after_rejected_request(
    client_script: dict, monkeypatch
) -> None:
    """
    A rejected request must not be queued or close the command reader.
    """
    session = client_script["GoalSession"](context=Mock(), feedback=Mock(), pool=Mock())
    send = Mock(side_effect=[False, True])
    monkeypatch.setattr(session, "send", send)
    monkeypatch.setattr(client_script["sys"], "stdin", StringIO("0.30\n0.40\nquit\n"))
    session.interact()
    assert [call.args[0] for call in send.call_args_list] == [0.3, 0.4]


def test_releasing_joy_lock_enables_preflight_again(feedback) -> None:
    """
    A fresh request may pass after the operator releases the interlock.
    """
    feedback.receive_joy(Bool(data=True))
    assert feedback.ready() is False
    feedback.receive_joy(Bool(data=False))
    assert feedback.ready() is True
