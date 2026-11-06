# %% imports
from __future__ import annotations

import os
import select
import signal
import subprocess
from contextlib import ExitStack
from pathlib import Path

import pytest

from test.experiments_test.test_tiago_goal_command import command
from test.experiments_test.test_tiago_giskard_safety import client_script


# %% shell command with a fake client, no ROS
@pytest.fixture
def request_pipe(tmp_path: Path):
    """
    Keep a temporary command pipe open without any robot process.
    """
    path = tmp_path / "requests.fifo"
    os.mkfifo(path)
    with ExitStack() as cleanup:
        descriptor = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        cleanup.callback(os.close, descriptor)
        yield path, descriptor


def test_ctrl_c_cancels_only_the_owned_goal(command: Path, request_pipe) -> None:
    """
    SIGINT must send the request ID and wait for cancellation confirmation.
    """
    path, descriptor = request_pipe
    sender = subprocess.Popen(
        ["bash", str(command), "0.5", "--fifo", str(path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    with ExitStack() as cleanup:
        cleanup.callback(sender.kill)
        assert select.select([descriptor], [], [], 3)[0] == [descriptor]
        fields = os.read(descriptor, 4096).decode().split()
        assert fields[0] == "request"
        assert fields[2] == "0.5"
        reply = Path(fields[3])
        reply.write_text("accepted\n")
        sender.send_signal(signal.SIGINT)
        assert select.select([descriptor], [], [], 3)[0] == [descriptor]
        assert os.read(descriptor, 4096).decode() == f"cancel {fields[1]}\n"
        assert sender.poll() is None
        reply.write_text("canceled\n")
        sender.communicate(timeout=5)
        assert sender.returncode == 130


def test_foreground_returns_when_goal_succeeds(command: Path, request_pipe) -> None:
    """
    The command must finish on success without requiring Ctrl+C.
    """
    path, descriptor = request_pipe
    sender = subprocess.Popen(
        ["bash", str(command), "0.3", "--fifo", str(path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    with ExitStack() as cleanup:
        cleanup.callback(sender.kill)
        assert select.select([descriptor], [], [], 3)[0] == [descriptor]
        fields = os.read(descriptor, 4096).decode().split()
        assert fields[0] == "request"
        Path(fields[3]).write_text("succeeded\n")
        sender.communicate(timeout=5)
        assert sender.returncode == 0


def test_targeted_cancel_does_not_cancel_another_request(
    client_script: dict, request_pipe
) -> None:
    """
    A delayed cancellation from a previous CLI must not stop a later goal.
    """
    path, descriptor = request_pipe
    os.write(descriptor, b"cancel previous-request\n")
    assert (
        client_script["GoalInbox"](path).poll_cancel(
            descriptor, active_id="current-request"
        )
        is False
    )
