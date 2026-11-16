"""
Cleanup success must not depend on a terminal remaining writable.
"""

# %% imports
import os
import shutil
import signal
import subprocess
from contextlib import ExitStack
from pathlib import Path


# %% lost display
def test_closed_terminal_does_not_turn_cleanup_success_into_failure(
    tmp_path: Path,
) -> None:
    """
    A successful Docker cleanup releases registration even with closed stdout.
    """
    root = Path(__file__).parents[3]
    binary = tmp_path / "bin"
    binary.mkdir()
    docker = binary / "docker"
    shutil.copy(
        Path(__file__).with_name("dataset") / "closed_terminal_docker.py", docker
    )
    docker.chmod(0o755)
    run_directory = tmp_path / "robocat-test-closed"
    run_directory.mkdir()
    record = tmp_path / "session.active"
    record.write_text("robocat-elevator")
    environment = os.environ.copy()
    environment["PATH"] = str(binary) + os.pathsep + environment["PATH"]
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
            stderr=subprocess.DEVNULL,
            text=True,
            start_new_session=True,
        )
        cleanup.callback(worker.wait, timeout=5)
        cleanup.callback(worker.terminate)
        assert worker.stdout.readline().strip() == "TEST TERMINAL READY"
        worker.stdout.close()
        os.killpg(worker.pid, signal.SIGHUP)
        assert worker.wait(timeout=5) == 129
        assert record.exists() is False
        assert (run_directory / "snm.cleanup_failed").exists() is False
        assert (run_directory / "snm.closed").exists() is True
        assert (run_directory / "cleanup.log").read_text().splitlines() == [
            "SNM session processes stopped; infrastructure retained."
        ]
