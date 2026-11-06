# %% imports
from __future__ import annotations

import os
import runpy
import select
import subprocess
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import Mock

import pytest

from test.experiments_test.test_tiago_goal_foreground import request_pipe
from test.experiments_test.test_tiago_goal_command import command


# %% automatic Ctrl+C through the existing sender
def test_cancel_timer_starts_on_movement_feedback(command: Path, request_pipe) -> None:
    """
    Cancel via SIGINT after movement, rather than after a guessed startup delay.
    """
    path, descriptor = request_pipe
    sender = subprocess.Popen(
        ["bash", str(command), "0.4", "--cancel-after", "0.01", "--fifo", str(path)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    with ExitStack() as cleanup:
        cleanup.callback(sender.kill)
        assert select.select([descriptor], [], [], 3)[0] == [descriptor]
        fields = os.read(descriptor, 4096).decode().split()
        assert fields[0] == "request"
        reply = Path(fields[3])
        reply.write_text("accepted\n")
        assert select.select([descriptor], [], [], 0.2)[0] == []
        reply.write_text("moving\n")
        assert select.select([descriptor], [], [], 3)[0] == [descriptor]
        assert os.read(descriptor, 4096).decode() == f"cancel {fields[1]}\n"
        reply.write_text("canceled\n")
        sender.communicate(timeout=5)
        assert sender.returncode == 130


# %% orchestration without robot processes
@pytest.fixture(scope="module")
def sequence_script() -> dict:
    """
    Load the sequence without starting subprocesses or sending goals.
    """
    return runpy.run_path(
        str(Path(__file__).parents[2] / "scripts/tiago_goal_sequence.py")
    )


def test_wait_for_stop_before_second_goal(sequence_script: dict, monkeypatch) -> None:
    """
    A blocked readiness check must delay the second goal.
    """
    sequence = sequence_script["GoalSequence"](
        first_distance=0.4, second_distance=0.4, cancel_after=0.3
    )
    commands = Mock(side_effect=[130, 3, 0, 0])
    monkeypatch.setattr(sequence, "run_command", commands)
    monkeypatch.setattr(sequence_script["time"], "sleep", Mock())
    sequence.run()
    assert [call.args[0] for call in commands.call_args_list] == [
        ["0.4", "--cancel-after", "0.3"],
        ["--ready"],
        ["--ready"],
        ["0.4"],
    ]


def test_no_second_goal_if_first_was_not_canceled(
    sequence_script: dict, monkeypatch
) -> None:
    """
    A first goal that finishes normally must not trigger the next movement.
    """
    sequence = sequence_script["GoalSequence"]()
    commands = Mock(return_value=0)
    monkeypatch.setattr(sequence, "run_command", commands)
    with pytest.raises(RuntimeError):
        sequence.run()
    assert commands.call_count == 1


def test_no_second_goal_without_stop_confirmation(
    sequence_script: dict, monkeypatch
) -> None:
    """
    Stop the sequence when wheels/interlocks never become ready.
    """
    sequence = sequence_script["GoalSequence"](ready_timeout=0)
    commands = Mock(side_effect=[130, 3])
    monkeypatch.setattr(sequence, "run_command", commands)
    with pytest.raises(TimeoutError):
        sequence.run()
    assert commands.call_count == 2
