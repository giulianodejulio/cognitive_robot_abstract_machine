# %% imports
from __future__ import annotations

import os
import subprocess
import time
from contextlib import ExitStack
from pathlib import Path


# %% a short log line must be visible while both children remain running
def test_launcher_shows_child_logs_before_exit(tmp_path: Path) -> None:
    """
    Pipe buffering must not hide readiness and short diagnostic lines until EOF.
    """
    root = Path(__file__).parents[2]
    helper = root / "test/experiments_test/dataset/tiago_launcher_child.py"
    environment = os.environ.copy()
    environment.update(TIAGO_PYTHON=str(helper), TIAGO_TEST_EVENT_DIR=str(tmp_path))
    output = tmp_path / "launcher.log"
    with output.open("w") as stream, ExitStack() as cleanup:
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
        cleanup.callback(launcher.wait, timeout=25)
        cleanup.callback(launcher.terminate)
        deadline = time.monotonic() + 12
        while "STACK READY" not in output.read_text() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert "STACK READY" in output.read_text()
        deadline = time.monotonic() + 3
        while "READY CLIENT:" not in output.read_text() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert launcher.poll() is None
        assert "READY CLIENT:" in output.read_text()
        diagnostic = "GOAL ERROR: phase=tracking; longitudinal=+0.0200 m\n"
        with (tmp_path / "logs/client.log").open("a") as client_log:
            client_log.write(diagnostic)
        deadline = time.monotonic() + 3
        while diagnostic not in output.read_text() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert diagnostic in output.read_text()
