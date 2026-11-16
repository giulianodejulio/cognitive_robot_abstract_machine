"""
Keep unresolved test clients visible across temporary session restarts.
"""

# %% imports
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest


# %% prepared persistent infrastructure
@pytest.fixture
def registered_infrastructure(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """
    Provide successful one-shot preparation and live persistent services.
    """
    dataset = Path(__file__).with_name("dataset")
    binary = tmp_path / "bin"
    binary.mkdir()
    for command, source in (
        ("byobu-tmux", "persistent_session_peer.py"),
        ("docker", "test_session_docker.py"),
    ):
        destination = binary / command
        shutil.copy(dataset / source, destination)
        destination.chmod(0o755)
    infra = tmp_path / "infra"
    infra.mkdir()
    for role in ("prepare", "dds", "giskard"):
        (infra / f"{role}.ready").touch()
    (infra / "prepare.closed").touch()
    monkeypatch.setenv("PATH", str(binary) + os.pathsep + os.environ["PATH"])
    monkeypatch.setenv("ROBOCAT_INFRA_DIRECTORY", str(infra))
    monkeypatch.setenv("ROBOCAT_BYOBU_LOG", str(tmp_path / "byobu.jsonl"))
    monkeypatch.setenv("ROBOCAT_WINDOW_LOG", str(tmp_path / "docker.jsonl"))
    return infra


def test_clean_test_session_registers_only_temporary_windows(
    registered_infrastructure: Path, tmp_path: Path
) -> None:
    """
    Closing preparation successfully does not prevent a registered test session.
    """
    root = Path(__file__).parents[3]
    result = subprocess.run(
        ["bash", str(root / "scripts/robocat-byobu"), "--no-attach"],
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 0, result.stderr
    commands = [
        json.loads(line) for line in (tmp_path / "byobu.jsonl").read_text().splitlines()
    ]
    windows = [
        command for command in commands if command[0] in ("new-session", "new-window")
    ]
    assert [command[command.index("-n") + 1] for command in windows] == ["snm", "robot"]
    records = list((registered_infrastructure / "test_sessions").glob("*.active"))
    assert len(records) == 1
    assert records[0].read_text().strip() == "robocat-elevator"


def test_unconfirmed_previous_cleanup_blocks_new_test_session(
    registered_infrastructure: Path, tmp_path: Path
) -> None:
    """
    A disappeared terminal with unresolved processes cannot be silently replaced.
    """
    directory = registered_infrastructure / "test_sessions"
    directory.mkdir()
    record = directory / "robocat-test-previous.active"
    record.write_text("previous-test")
    root = Path(__file__).parents[3]
    result = subprocess.run(
        ["bash", str(root / "scripts/robocat-byobu"), "--no-attach"],
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 1
    commands = [
        json.loads(line) for line in (tmp_path / "byobu.jsonl").read_text().splitlines()
    ]
    assert [
        command for command in commands if command[0] in ("new-session", "new-window")
    ] == []
    assert record.read_text() == "previous-test"


def test_infrastructure_stop_refuses_unconfirmed_test_cleanup(
    registered_infrastructure: Path, tmp_path: Path
) -> None:
    """
    Persistent services cannot be stopped while a closed client's cleanup is unresolved.
    """
    directory = registered_infrastructure / "test_sessions"
    directory.mkdir()
    record = directory / "robocat-test-previous.active"
    record.write_text("previous-test")
    root = Path(__file__).parents[3]
    result = subprocess.run(
        ["bash", str(root / "scripts/robocat-infra"), "stop"],
        input="STOP\n",
        capture_output=True,
        text=True,
        timeout=5,
    )
    assert result.returncode == 1
    assert (tmp_path / "docker.jsonl").exists() is False
    assert record.read_text() == "previous-test"
