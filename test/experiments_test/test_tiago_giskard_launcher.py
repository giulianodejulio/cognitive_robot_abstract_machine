# %% imports
from __future__ import annotations

import os
import subprocess
import time
from pathlib import Path


# %% process supervision without ROS nodes
def test_launcher_help_does_not_start_children() -> None:
    """
    Request usage without starting a controller or client.
    """
    script = Path(__file__).parents[2] / "scripts/launch_tiago_giskard.sh"
    result = subprocess.run(
        ["bash", str(script), "--help"], capture_output=True, text=True, timeout=5
    )
    assert result.returncode == 0
    assert "--fifo" in result.stdout


def test_launcher_waits_for_both_and_stops_owned_children(tmp_path: Path) -> None:
    """
    Supervise fake children without starting a real Giskard process.
    """
    root = Path(__file__).parents[2]
    helper = root / "test/experiments_test/dataset/tiago_launcher_child.py"
    environment = os.environ.copy()
    environment.update(TIAGO_PYTHON=str(helper), TIAGO_TEST_EVENT_DIR=str(tmp_path))
    output = tmp_path / "launcher.log"
    with output.open("w") as stream:
        launcher = subprocess.Popen(
            [
                "bash",
                str(root / "scripts/launch_tiago_giskard.sh"),
                "--fifo",
                str(tmp_path / "goal.fifo"),
                "--log-dir",
                str(tmp_path / "logs"),
                "--startup-timeout",
                "10",
            ],
            stdout=stream,
            stderr=subprocess.STDOUT,
            env=environment,
        )
        deadline = time.monotonic() + 15
        while (
            launcher.poll() is None
            and "STACK READY" not in output.read_text()
            and time.monotonic() < deadline
        ):
            time.sleep(0.05)
        launcher.terminate()
        launcher.wait(timeout=25)
    assert "STACK READY" in output.read_text()
    assert (tmp_path / "server.stopped").read_text() == "terminated"
    assert (tmp_path / "client.stopped").read_text() == "terminated"
