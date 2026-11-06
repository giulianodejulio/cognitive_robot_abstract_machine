# %% imports
from __future__ import annotations

import signal
import subprocess
import time
from contextlib import ExitStack
from pathlib import Path
from threading import Event, Thread
from unittest.mock import Mock

from test.experiments_test.test_tiago_giskard_safety import client_script
from test.experiments_test.test_tiago_goal_command import command


# %% complete local protocol with a fake motion executor
def test_cancel_then_new_request_keeps_client_alive(
    client_script: dict, command: Path, tmp_path: Path
) -> None:
    """
    Exercise CLI, FIFO, targeted cancel and reuse without any ROS action.
    """
    path = tmp_path / "requests.fifo"
    ready = Event()
    started = Event()
    session = Mock()
    session.report_status.side_effect = ready.set

    def execute(distance: float, requested_at: float, cancel_requested) -> bool:
        """
        Stand in for an action that waits for its cancellation command.
        """
        if distance == 0.5:
            started.set()
            deadline = time.monotonic() + 5
            while not cancel_requested() and time.monotonic() < deadline:
                time.sleep(0.01)
            session.last_outcome = "canceled"
            return False
        return True

    session.send.side_effect = execute
    inbox = client_script["GoalInbox"](path)
    receiver = Thread(target=inbox.serve, args=(session,), daemon=True)
    receiver.start()
    assert ready.wait(timeout=3) is True
    sender = subprocess.Popen(
        ["bash", str(command), "0.5", "--fifo", str(path)],
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
        result = subprocess.run(
            ["bash", str(command), "0.3", "--fifo", str(path)],
            capture_output=True,
            text=True,
            timeout=5,
        )
        assert result.returncode == 0
        assert receiver.is_alive() is True
        path.write_text("quit\n")
        receiver.join(timeout=3)
        assert receiver.is_alive() is False
