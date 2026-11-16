"""
A closed terminal requests cleanup of its owned container server.
"""

# %% imports
import json
import os
import shutil
import signal
import subprocess
import time
from contextlib import ExitStack
from pathlib import Path


# %% terminal hangup
def test_giskard_window_hangup_requests_owned_cleanup(tmp_path: Path) -> None:
    """
    Session termination closes the owned server instead of only Docker attach.
    """
    root = Path(__file__).parents[3]
    binary = tmp_path / "bin"
    binary.mkdir()
    docker = binary / "docker"
    shutil.copy(Path(__file__).with_name("dataset") / "window_docker.py", docker)
    docker.chmod(0o755)
    run_directory = tmp_path / "robocat-byobu-test"
    run_directory.mkdir()
    for dependency in ("prepare", "dds"):
        (run_directory / f"{dependency}.ready").touch()
    log = tmp_path / "docker.jsonl"
    environment = os.environ.copy()
    environment["PATH"] = str(binary) + os.pathsep + environment["PATH"]
    environment["ROBOCAT_WINDOW_LOG"] = str(log)
    with ExitStack() as cleanup:
        worker = subprocess.Popen(
            [
                "bash",
                str(root / "scripts/robocat_byobu_window.sh"),
                "giskard",
                str(run_directory),
                "0.1",
                "0.2",
            ],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        cleanup.callback(worker.wait, timeout=5)
        cleanup.callback(worker.terminate)
        deadline = time.monotonic() + 5
        while (
            not (run_directory / "giskard.ready").exists()
            and time.monotonic() < deadline
        ):
            time.sleep(0.05)
        assert (run_directory / "giskard.ready").exists()
        os.killpg(worker.pid, signal.SIGHUP)
        assert worker.wait(timeout=5) == 129
        commands = [json.loads(line) for line in log.read_text().splitlines()]
        assert commands[-1] == [
            "exec",
            "-w",
            "/workspace/cram",
            "aicor-cram-demo",
            "bash",
            "scripts/robocat_stop_owned_process.sh",
            f"/tmp/{run_directory.name}.giskard.pid",
            run_directory.name,
        ]
