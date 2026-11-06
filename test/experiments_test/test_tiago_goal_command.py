# %% imports
from __future__ import annotations

import os
import subprocess
from pathlib import Path
from threading import Thread

import pytest


# %% lightweight command transport
@pytest.fixture
def command() -> Path:
    """
    Locate the sender without importing robot control libraries.
    """
    return Path(__file__).parents[2] / "scripts/tiago-goal"


def test_send_distance_to_existing_reader(command: Path, tmp_path: Path) -> None:
    """
    Forward the requested distance to the already running client.
    """
    fifo = tmp_path / "goals.fifo"
    os.mkfifo(fifo)
    received = []
    reader = Thread(target=lambda: received.append(fifo.read_text()), daemon=True)
    reader.start()
    result = subprocess.run(
        ["bash", str(command), "0.75", "--no-wait", "--fifo", str(fifo)],
        capture_output=True,
        text=True,
        timeout=5,
    )
    reader.join(timeout=3)
    assert result.returncode == 0
    assert received == ["0.75\n"]


@pytest.mark.parametrize("distance", ["0", "-0.3", "nan", "inf", "1e9999", "abc"])
def test_reject_invalid_distance(command: Path, distance: str) -> None:
    """
    Reject invalid requests without writing to a client pipe.
    """
    result = subprocess.run(
        ["bash", str(command), distance], capture_output=True, text=True, timeout=5
    )
    assert result.returncode == 2


def test_missing_client_does_not_create_file(command: Path, tmp_path: Path) -> None:
    """
    A missing client must produce an error without leaving a regular file.
    """
    fifo = tmp_path / "absent.fifo"
    result = subprocess.run(
        ["bash", str(command), "0.3", "--fifo", str(fifo)],
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 1
    assert fifo.exists() is False


def test_stale_fifo_does_not_block(command: Path, tmp_path: Path) -> None:
    """
    Return promptly when a leftover pipe has no client reader.
    """
    fifo = tmp_path / "stale.fifo"
    os.mkfifo(fifo)
    result = subprocess.run(
        ["bash", str(command), "0.3", "--fifo", str(fifo)],
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 1
