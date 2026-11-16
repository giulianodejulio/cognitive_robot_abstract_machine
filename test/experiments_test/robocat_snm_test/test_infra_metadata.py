"""
Infrastructure startup must register its run directory on compatible tmux targets.
"""

# %% imports
import os
import shutil
import subprocess
from pathlib import Path


# %% session metadata
def test_start_records_metadata_without_unsupported_target_prefix(
    tmp_path: Path,
) -> None:
    """
    A successful server startup must also expose its directory to test sessions.
    """
    binary = tmp_path / "bin"
    binary.mkdir()
    for name in ("byobu-tmux", "docker"):
        target = binary / name
        shutil.copy(
            Path(__file__).with_name("dataset") / "infra_metadata_tmux.sh", target
        )
        target.chmod(0o755)
    log = tmp_path / "commands.log"
    environment = {
        **os.environ,
        "PATH": str(binary) + os.pathsep + os.environ["PATH"],
        "ROBOCAT_METADATA_LOG": str(log),
    }
    root = Path(__file__).parents[3]
    result = subprocess.run(
        ["bash", str(root / "scripts/robocat-infra"), "start"],
        env=environment,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    commands = log.read_text().splitlines()
    assert any(
        line.startswith(
            "set-option -t robocat-infra @robocat_run_directory /tmp/robocat-infra."
        )
        for line in commands
    )
    assert "set-option -t robocat-infra @robocat_infrastructure 1" in commands
