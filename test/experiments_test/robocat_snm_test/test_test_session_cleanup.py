"""
Close a test client's process tree without touching persistent services.
"""

# %% imports
import os
import subprocess
import sys
from contextlib import ExitStack
from pathlib import Path

import pytest


# %% owned request cancellation
@pytest.mark.parametrize("matching_owner", [True, False])
def test_test_session_cancels_only_its_owned_request(
    tmp_path: Path, matching_owner: bool
) -> None:
    """
    Session cleanup interrupts a child request while unrelated processes survive.
    """
    root = Path(__file__).parents[3]
    dataset = Path(__file__).with_name("dataset")
    owner = tmp_path.name
    environment = os.environ.copy()
    environment["ROBOCAT_BYOBU_OWNER"] = owner
    with ExitStack() as cleanup:
        unrelated = subprocess.Popen(
            [sys.executable, str(dataset / "owned_process.py")],
            stdout=subprocess.PIPE,
            text=True,
        )
        cleanup.callback(unrelated.wait, timeout=5)
        cleanup.callback(unrelated.terminate)
        assert unrelated.stdout.readline().strip() == "READY"
        session = subprocess.Popen(
            ["bash", str(dataset / "owned_test_shell.sh")],
            env=environment,
            stdout=subprocess.PIPE,
            text=True,
        )
        cleanup.callback(session.wait, timeout=5)
        cleanup.callback(session.terminate)
        assert session.stdout.readline().strip() == "READY"
        pidfile = tmp_path / "snm.pid"
        pidfile.write_text(str(session.pid))
        result = subprocess.run(
            [
                "bash",
                str(root / "scripts/robocat_stop_test_session.sh"),
                str(pidfile),
                owner if matching_owner else owner + "-other",
            ],
            capture_output=True,
            text=True,
            timeout=25,
        )
        assert unrelated.poll() is None
        if matching_owner:
            assert result.returncode == 0, result.stderr
            assert session.wait(timeout=3) == 0
            assert session.stdout.read().strip() == "INTERRUPTED"
            assert pidfile.exists() is False
        else:
            assert result.returncode == 1
            assert session.poll() is None
            session.terminate()
            # Release the owned child through the verified cleanup path.
            subprocess.run(
                [
                    "bash",
                    str(root / "scripts/robocat_stop_test_session.sh"),
                    str(pidfile),
                    owner,
                ],
                capture_output=True,
                timeout=25,
            )
