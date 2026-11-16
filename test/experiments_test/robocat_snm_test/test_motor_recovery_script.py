"""
Guided recovery preserves settings and stops on failed evidence.
"""

# %% imports
import json
import os
import shutil
import socket
import subprocess
from pathlib import Path

import pytest


# %% isolated command environment
@pytest.fixture
def recovery_environment(tmp_path: Path) -> dict[str, str]:
    """
    Provide isolated infrastructure, a socket and substitutes for all remote tools.
    """
    root = Path(__file__).parents[3]
    for script in (
        "robocat-recover-motors",
        "connect_robocat_planner.sh",
        "prepare_robocat_controller.sh",
    ):
        target = tmp_path / "scripts" / script
        target.parent.mkdir(exist_ok=True)
        shutil.copy(root / "scripts" / script, target)
    sources = root / "experiments/src/experiments"
    for name in (
        "__init__.py",
        "_version.py",
        "robocat_snm/__init__.py",
        "robocat_snm/native_navigation.py",
        "robocat_snm/native_preparation.py",
        "robocat_snm/motor_recovery.py",
        "robocat_snm/recovery_remote.sh",
        "robocat_snm/preparation_remote.sh",
    ):
        target = tmp_path / "experiments/src/experiments" / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(sources / name, target)
    binary = tmp_path / "bin"
    binary.mkdir()
    shutil.copy(Path(__file__).with_name("dataset") / "recovery_arguments.sh", binary)
    for name in ("ssh", "docker", "byobu-tmux"):
        target = binary / name
        shutil.copy(Path(__file__).with_name("dataset") / "recovery_command.py", target)
        target.chmod(0o755)
    infrastructure = tmp_path / "infra"
    infrastructure.mkdir()
    for name in ("prepare.ready", "dds.ready", "giskard.ready"):
        (infrastructure / name).touch()
    connection = tmp_path / ".robocat-planner"
    connection.mkdir()
    with socket.socket(socket.AF_UNIX) as endpoint:
        endpoint.bind(str(connection / "ssh.sock"))
    return {
        **os.environ,
        "ROBOCAT_RECOVERY_TEST": str(tmp_path),
        "PATH": str(binary) + os.pathsep + os.environ["PATH"],
    }


# %% guided execution
@pytest.mark.parametrize("restart", ["yes", "no"])
def test_recovery_preserves_settings_and_restarts_only_if_needed(
    recovery_environment, restart
) -> None:
    """
    The software sequence retains speed/tolerance settings and never sends a goal.
    """
    environment = {**recovery_environment, "ROBOCAT_RECOVERY_RESTART": restart}
    root = Path(environment["ROBOCAT_RECOVERY_TEST"])
    result = subprocess.run(
        ["bash", str(root / "scripts/robocat-recover-motors")],
        input="STOPPED\nPRESSED\nOK\n",
        text=True,
        capture_output=True,
        env=environment,
        timeout=20,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "RECOVERY OK:" in result.stdout
    commands = [
        json.loads(line) for line in (root / "commands.jsonl").read_text().splitlines()
    ]
    remote_commands = [
        shlex_command[-1] for name, shlex_command in commands if name == "ssh"
    ]
    assert any(
        "restore --position 0.03 --orientation 0.0872665" in item
        for item in remote_commands
    )
    restarts = [
        item for item in remote_commands if item.endswith("restart controller_manager ")
    ]
    assert len(restarts) == (1 if restart == "yes" else 0)
    respawn = [
        arguments
        for name, arguments in commands
        if name == "byobu-tmux" and arguments[0] == "respawn-pane"
    ]
    assert len(respawn) == 1
    assert respawn[0][-1].endswith(" 0.15 0.2 ")
    assert all("send_goal" not in item for item in remote_commands)


@pytest.mark.parametrize(
    "failure", ["stopped", "pressed", "rearmed", "pal", "preparation", "restore"]
)
def test_failed_recovery_never_restarts_giskard(recovery_environment, failure) -> None:
    """
    Each failed recovery boundary prevents a misleading ready message or restart.
    """
    environment = {
        **recovery_environment,
        "ROBOCAT_RECOVERY_RESTART": "yes",
        "ROBOCAT_RECOVERY_FAIL": failure,
    }
    root = Path(environment["ROBOCAT_RECOVERY_TEST"])
    result = subprocess.run(
        ["bash", str(root / "scripts/robocat-recover-motors")],
        input="STOPPED\nPRESSED\nOK\n",
        text=True,
        capture_output=True,
        env=environment,
        timeout=20,
    )
    assert result.returncode != 0
    assert "RECOVERY OK:" not in result.stdout
    commands = [
        json.loads(line) for line in (root / "commands.jsonl").read_text().splitlines()
    ]
    assert [
        (name, arguments[0])
        for name, arguments in commands
        if name == "byobu-tmux" and arguments[0] == "respawn-pane"
    ] == []


def test_old_server_ready_log_is_not_reused(recovery_environment) -> None:
    """
    A historical READY line cannot certify the replacement server.
    """
    root = Path(recovery_environment["ROBOCAT_RECOVERY_TEST"])
    log = root / "infra/giskard.log"
    log.write_text("READY: historical server\n")
    result = subprocess.run(
        ["bash", str(root / "scripts/robocat-recover-motors")],
        input="STOPPED\nPRESSED\nOK\n",
        text=True,
        capture_output=True,
        env=recovery_environment,
        timeout=20,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert log.read_text() == ""
