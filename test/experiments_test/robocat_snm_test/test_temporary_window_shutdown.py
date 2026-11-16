"""
Keep infrastructure alive while closing a temporary session's client.
"""

# %% imports
import json
import os
import shutil
import signal
import subprocess
from contextlib import ExitStack
from pathlib import Path

import pytest


# %% test window hangup
@pytest.mark.parametrize("cleanup_result", [0, 1])
def test_temporary_window_hangup_tracks_cleanup_result(
    tmp_path: Path, cleanup_result: int
) -> None:
    """
    Only confirmed cleanup releases the registration for future test sessions.
    """
    root = Path(__file__).parents[3]
    binary = tmp_path / "bin"
    binary.mkdir()
    docker = binary / "docker"
    shutil.copy(Path(__file__).with_name("dataset") / "test_session_docker.py", docker)
    docker.chmod(0o755)
    run_directory = tmp_path / "robocat-test-owned"
    run_directory.mkdir()
    record = tmp_path / "session.active"
    record.write_text("robocat-elevator")
    log = tmp_path / "docker.jsonl"
    environment = os.environ.copy()
    environment["PATH"] = str(binary) + os.pathsep + environment["PATH"]
    environment["ROBOCAT_WINDOW_LOG"] = str(log)
    environment["ROBOCAT_CLEANUP_RESULT"] = str(cleanup_result)
    with ExitStack() as cleanup:
        worker = subprocess.Popen(
            [
                "bash",
                str(root / "scripts/robocat_test_window.sh"),
                "snm",
                str(run_directory),
                str(record),
            ],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        cleanup.callback(worker.wait, timeout=5)
        cleanup.callback(worker.terminate)
        assert worker.stdout.readline().strip() == "TEST TERMINAL READY"
        os.killpg(worker.pid, signal.SIGHUP)
        assert worker.wait(timeout=5) == (129 if cleanup_result == 0 else 1)
        commands = [json.loads(line) for line in log.read_text().splitlines()]
        assert commands == [
            [
                "exec",
                "-e",
                f"ROBOCAT_BYOBU_OWNER={run_directory.name}",
                "-w",
                "/workspace/cram",
                "-it",
                "aicor-cram-demo",
                "bash",
                "scripts/robocat_session_container.sh",
                "snm",
            ],
            [
                "exec",
                "-w",
                "/workspace/cram",
                "aicor-cram-demo",
                "bash",
                "scripts/robocat_stop_test_session.sh",
                f"/tmp/{run_directory.name}.snm.pid",
                run_directory.name,
            ],
        ]
        assert record.exists() is (cleanup_result != 0)
        assert (run_directory / "snm.cleanup_failed").exists() is (cleanup_result != 0)
