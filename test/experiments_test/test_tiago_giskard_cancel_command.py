# %% imports
from __future__ import annotations

import os
import subprocess
from contextlib import ExitStack
from pathlib import Path
from threading import Thread

from test.experiments_test.test_tiago_giskard_safety import client_script
from test.experiments_test.test_tiago_goal_command import command


# %% cancellation transport without ROS
def test_cancel_is_read_while_busy(client_script: dict, tmp_path: Path) -> None:
    """
    Read cancellation during a running goal and discard other queued distances.
    """
    path = tmp_path / "control.fifo"
    os.mkfifo(path)
    with ExitStack() as cleanup:
        descriptor = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        cleanup.callback(os.close, descriptor)
        os.write(descriptor, b"cancel\n0.8\n")
        inbox = client_script["GoalInbox"](path)
        assert inbox.poll_cancel(descriptor) is True
        assert inbox.poll_cancel(descriptor) is False


def test_shell_cancel_reaches_existing_client(command: Path, tmp_path: Path) -> None:
    """
    The lightweight sender must deliver cancellation without a ROS startup.
    """
    path = tmp_path / "control.fifo"
    os.mkfifo(path)
    received = []
    reader = Thread(target=lambda: received.append(path.read_text()), daemon=True)
    reader.start()
    result = subprocess.run(
        ["bash", str(command), "--cancel", "--fifo", str(path)],
        capture_output=True,
        text=True,
        timeout=5,
    )
    reader.join(timeout=3)
    assert result.returncode == 0
    assert received == ["cancel\n"]
