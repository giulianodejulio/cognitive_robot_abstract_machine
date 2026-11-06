# %% imports
from __future__ import annotations

import os
from pathlib import Path
from threading import Thread
from unittest.mock import Mock

import pytest

from test.experiments_test.test_tiago_giskard_safety import client_script


# %% local transport without ROS or movement
def test_fifo_dispatches_request_and_cleans_up_on_failure(
    client_script: dict, tmp_path: Path
) -> None:
    """
    A shell-style FIFO write reaches the existing session, with safe cleanup.
    """
    path = tmp_path / "requests.fifo"
    os.mkfifo(path)
    session = Mock()
    session.parse_distance.side_effect = client_script["GoalSession"].parse_distance
    session.send.side_effect = RuntimeError("test goal failure")
    writer = Thread(target=path.write_text, args=("0.30\n",), daemon=True)
    writer.start()
    with pytest.raises(RuntimeError, match="test goal failure"):
        client_script["GoalInbox"](path).serve(session)
    writer.join(timeout=5)
    assert writer.is_alive() is False
    assert session.send.call_count == 1
    assert session.send.call_args.args == (0.3,)
    assert path.exists() is False


def test_quit_closes_fifo_without_sending_a_goal(
    client_script: dict, tmp_path: Path
) -> None:
    """
    Closing the warmed client must not execute a default motion.
    """
    path = tmp_path / "requests.fifo"
    os.mkfifo(path)
    session = Mock()
    writer = Thread(target=path.write_text, args=("quit\n",), daemon=True)
    writer.start()
    client_script["GoalInbox"](path).serve(session)
    writer.join(timeout=5)
    assert writer.is_alive() is False
    session.send.assert_not_called()
    assert path.exists() is False


def test_existing_file_is_preserved(client_script: dict, tmp_path: Path) -> None:
    """
    Refuse an existing ordinary file rather than replacing local data.
    """
    path = tmp_path / "requests.fifo"
    path.write_text("preserve me")
    with pytest.raises(ValueError):
        client_script["GoalInbox"](path).serve(Mock())
    assert path.read_text() == "preserve me"
