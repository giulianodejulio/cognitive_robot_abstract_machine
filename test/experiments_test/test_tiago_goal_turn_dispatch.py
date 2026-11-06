# %% imports
from __future__ import annotations

import signal
import subprocess
import time
from contextlib import ExitStack
from pathlib import Path
from threading import Event, Thread
from unittest.mock import Mock

import pytest

from test.experiments_test.test_tiago_giskard_safety import client_script
from test.experiments_test.test_tiago_goal_command import command


# %% local turn cancellation through the persistent receiver
@pytest.mark.parametrize("expected_pure_turn", [False, True])
def test_turn_dispatch_and_owned_cancellation(
    client_script: dict,
    command: Path,
    tmp_path: Path,
    expected_pure_turn: bool,
) -> None:
    """
    Turn requests use the shared cancellation path and leave the receiver alive.
    """
    path = tmp_path / "requests.fifo"
    ready, started = Event(), Event()
    session = Mock()
    session.report_status.side_effect = ready.set
    session.feedback.onset.latency = None

    def execute(
        distance: float,
        requested_at: float,
        cancel_requested,
        turn_degrees: float = 0,
        pure_turn: bool = False,
    ) -> bool:
        """
        Wait for the owning sender's cancellation without executing an action.
        """
        assert distance == 0
        assert turn_degrees == 30
        assert pure_turn == expected_pure_turn
        started.set()
        deadline = time.monotonic() + 5
        while not cancel_requested() and time.monotonic() < deadline:
            time.sleep(0.01)
        session.last_outcome = "canceled"
        return False

    session.send.side_effect = execute
    receiver = Thread(
        target=client_script["GoalInbox"](path).serve,
        args=(session,),
        daemon=True,
    )
    receiver.start()
    assert ready.wait(timeout=3) is True
    arguments = ["bash", str(command), "--turn", "30", "--fifo", str(path)]
    if expected_pure_turn:
        arguments.append("--pure-turn")
    sender = subprocess.Popen(
        arguments,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    with ExitStack() as cleanup:
        cleanup.callback(sender.kill)
        assert started.wait(timeout=3) is True
        sender.send_signal(signal.SIGINT)
        sender.communicate(timeout=5)
        assert sender.returncode == 130
        assert receiver.is_alive() is True
        path.write_text("quit\n")
        receiver.join(timeout=3)
        assert receiver.is_alive() is False
